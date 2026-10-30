#include <M5Cardputer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <atomic>
#include <cstring>
#include <cstdlib>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>

#include "camera_frame.h"
#include "protocol.h"
#include "voice_filter.h"

namespace {
constexpr uint16_t kControlPort = 2000;
constexpr uint16_t kAudioPort = 8081;
constexpr uint16_t kCameraPort = 8080;
constexpr size_t kCameraFrameCapacity = 48 * 1024;
constexpr size_t kCameraReadChunk = 2048;
constexpr size_t kAudioSamples = 640;
constexpr uint32_t kAudioSampleRate = 11025;
constexpr size_t kMicReadSamples = 256;
constexpr size_t kMicCaptureSamples = kAudioSamples * 2;
constexpr int kSpeedPresets[] = {6, 12, 18, 24, 31};
constexpr uint32_t kSendIntervalMs = 150;
constexpr uint32_t kStatusTimeoutMs = 2000;
constexpr uint32_t kReconnectIntervalMs = 3000;
constexpr uint32_t kWifiConnectTimeoutMs = 12000;
constexpr size_t kMaxRobotNetworks = 8;
constexpr char kRobotHost[] = "192.168.10.123";
constexpr char kRobotSsidPrefix[] = "CL4P-TP";

enum class WifiPhase { Scanning, Choosing, Connecting, Connected, NoneFound, Failed };
struct RobotNetwork {
  char ssid[33];
  int rssi;
  bool open;
};

WiFiClient robot;
WiFiUDP audio;
RobotNetwork robotNetworks[kMaxRobotNetworks] = {};
char selectedSsid[33] = {};
WifiPhase wifiPhase = WifiPhase::Scanning;
size_t robotNetworkCount = 0;
size_t selectedNetwork = 0;
uint32_t scanStartedMs = 0;
uint32_t wifiConnectStartedMs = 0;
bool scanStarted = false;
int16_t audioSamples[kAudioSamples] = {};
int16_t micReadSamples[kMicReadSamples] = {};
int16_t micCaptureSamples[kMicCaptureSamples] = {};
claptrap::VoiceFilter voiceFilter;
size_t micCaptureCount = 0;
i2s_chan_handle_t micChannel = nullptr;
uint8_t statusBuffer[claptrap::kPacketSize] = {};
size_t statusLength = 0;
uint32_t lastStatusMs = 0;
uint32_t lastSendMs = 0;
uint32_t lastConnectAttemptMs = 0;
uint32_t lastDrawMs = 0;
int strength = 12;
std::atomic<int> displayedSpeed{12};
int throttle = 0;
int steering = 0;
bool driveReady = false;
bool enterWasDown = false;
bool leftBracketWasDown = false;
bool rightBracketWasDown = false;
bool micLowerWasDown = false;
bool micHigherWasDown = false;
bool videoWasDown = false;
bool navUpWasDown = false;
bool navDownWasDown = false;
bool rescanWasDown = false;
bool talking = false;
bool micReady = false;
enum class MicError { None, Socket, Start, Record, Send };
MicError micError = MicError::None;
std::atomic<uint16_t> audioPeak{0};
std::atomic<uint16_t> audioRange{0};
std::atomic<uint8_t> micOutputPercent{40};
uint32_t audioPacketsSent = 0;
std::atomic<bool> micButtonHeld{false};
std::atomic<bool> micRunning{false};
bool videoStartFailed = false;
bool screenNeedsClear = true;
bool cameraNeedsClear = true;
std::atomic<bool> videoRequested{false};
std::atomic<bool> videoRunning{false};

void cameraTask(void*);

void toggleVideo() {
  screenNeedsClear = true;
  if (videoRequested.load()) {
    videoRequested.store(false);
    return;
  }
  if (videoRunning.load()) return;  // Let the previous video task finish first.
  videoStartFailed = false;
  videoRequested.store(true);
  videoRunning.store(true);
  if (xTaskCreatePinnedToCore(cameraTask, "claptrap-video", 12288, nullptr, 1,
                              nullptr, 0) != pdPASS) {
    videoRequested.store(false);
    videoRunning.store(false);
    videoStartFailed = true;
  }
}

void stopMicrophone() {
  micRunning.store(false);
  if (!talking) return;
  audio.stop();
  talking = false;
  voiceFilter.release();
  micCaptureCount = 0;
  audioPeak = 0;
  audioRange = 0;
}

bool writeMicRegister(uint8_t reg, uint8_t value) {
  return M5.In_I2C.writeRegister8(0x18, reg, value, 100000);
}

bool beginMicrophone() {
  // Reset the shared audio pins before starting microphone capture.
  M5Cardputer.Speaker.end();
  for (gpio_num_t pin : {GPIO_NUM_41, GPIO_NUM_43, GPIO_NUM_46, GPIO_NUM_42}) {
    gpio_reset_pin(pin);
  }
  if (!writeMicRegister(0x00, 0x00)) return false;  // Restart the audio codec.
  delay(2);
  const uint8_t registers[][2] = {
      {0x00, 0x80}, {0x01, 0xBA}, {0x02, 0x18},
      {0x09, 0x0C}, {0x0A, 0x0C},  // Use 16-bit audio in both directions.
      {0x0D, 0x01}, {0x0E, 0x02}, {0x12, 0x02},  // Power the mic input and leave speaker output off.
      {0x14, 0x17}, {0x17, 0xCB}, {0x1C, 0x6A},
  };
  for (const auto& entry : registers) {
    if (!writeMicRegister(entry[0], entry[1])) return false;
  }
  i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 12;
  channelConfig.dma_frame_num = 256;
  if (i2s_new_channel(&channelConfig, nullptr, &micChannel) != ESP_OK) return false;
  i2s_std_config_t standardConfig = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kAudioSampleRate * 2),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = GPIO_NUM_41,
          .ws = GPIO_NUM_43,
          .dout = I2S_GPIO_UNUSED,
          .din = GPIO_NUM_46,
          .invert_flags = {},
      },
  };
  standardConfig.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
  if (i2s_channel_init_std_mode(micChannel, &standardConfig) != ESP_OK ||
      i2s_channel_enable(micChannel) != ESP_OK) {
    i2s_del_channel(micChannel);
    micChannel = nullptr;
    return false;
  }
  // Start the audio clock before enabling microphone capture.
  delay(10);
  if (writeMicRegister(0x0D, 0x01)) return true;
  i2s_channel_disable(micChannel);
  i2s_del_channel(micChannel);
  micChannel = nullptr;
  return false;
}

bool statusFresh(uint32_t now) {
  return lastStatusMs != 0 && now - lastStatusMs < kStatusTimeoutMs;
}

void sendAxes(int nextThrottle, int nextSteering) {
  if (!robot.connected()) return;
  uint8_t packet[claptrap::kPacketSize];
  claptrap::buildPacket(nextThrottle, nextSteering, packet);
  if (robot.write(packet, sizeof packet) != sizeof packet) {
    robot.stop();
    driveReady = false;
  }
  lastSendMs = millis();
}

void sendNeutral() {
  throttle = 0;
  steering = 0;
  sendAxes(0, 0);
}

void disconnectRobot() {
  stopMicrophone();
  sendNeutral();
  robot.stop();
  driveReady = false;
  lastStatusMs = 0;
  statusLength = 0;
}

void beginWifiScan() {
  screenNeedsClear = true;
  videoRequested.store(false);
  disconnectRobot();
  WiFi.disconnect();
  WiFi.scanDelete();
  robotNetworkCount = 0;
  selectedNetwork = 0;
  scanStarted = false;
  scanStartedMs = 0;
  wifiPhase = WifiPhase::Scanning;
}

void connectToSelectedRobot(uint32_t now) {
  if (selectedNetwork >= robotNetworkCount || !robotNetworks[selectedNetwork].open) return;
  strncpy(selectedSsid, robotNetworks[selectedNetwork].ssid, sizeof selectedSsid);
  selectedSsid[sizeof selectedSsid - 1] = '\0';
  WiFi.scanDelete();
  WiFi.begin(selectedSsid);
  wifiConnectStartedMs = now;
  wifiPhase = WifiPhase::Connecting;
}

void updateWifi(uint32_t now) {
  if (wifiPhase == WifiPhase::Scanning) {
    if (!scanStarted) {
      WiFi.scanNetworks(true, false);
      scanStarted = true;
      scanStartedMs = now;
      return;
    }
    const int count = WiFi.scanComplete();
    if (count == -1 && now - scanStartedMs < 15000) return;
    if (count >= 0) {
      for (int i = 0; i < count; ++i) {
        const String ssid = WiFi.SSID(i);
        if (!ssid.startsWith(kRobotSsidPrefix)) continue;
        if (robotNetworkCount >= kMaxRobotNetworks) break;
        RobotNetwork& network = robotNetworks[robotNetworkCount++];
        ssid.toCharArray(network.ssid, sizeof network.ssid);
        network.rssi = WiFi.RSSI(i);
        network.open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
      }
    }
    WiFi.scanDelete();
    wifiPhase = robotNetworkCount ? WifiPhase::Choosing : WifiPhase::NoneFound;
    return;
  }
  if (wifiPhase == WifiPhase::Connecting) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiPhase = WifiPhase::Connected;
      lastConnectAttemptMs = 0;
    } else if (now - wifiConnectStartedMs >= kWifiConnectTimeoutMs) {
      WiFi.disconnect();
      wifiPhase = WifiPhase::Failed;
    }
    return;
  }
  if (wifiPhase == WifiPhase::Connected && WiFi.status() != WL_CONNECTED) {
    disconnectRobot();
    videoRequested.store(false);
    screenNeedsClear = true;
    WiFi.begin(selectedSsid);
    wifiConnectStartedMs = now;
    wifiPhase = WifiPhase::Connecting;
  }
}

void ingestStatus(uint8_t byte, uint32_t now) {
  if (statusLength == 0 && byte != 0xee) return;
  statusBuffer[statusLength++] = byte;
  if (statusLength < sizeof statusBuffer) return;
  if (statusBuffer[21] == 0xdd) {
    lastStatusMs = now;
    statusLength = 0;
    return;
  }
  // A damaged frame may contain the start of the next frame.
  size_t next = 1;
  while (next < sizeof statusBuffer && statusBuffer[next] != 0xee) ++next;
  statusLength = sizeof statusBuffer - next;
  if (statusLength) memmove(statusBuffer, statusBuffer + next, statusLength);
}

void updateConnection(uint32_t now) {
  if (wifiPhase != WifiPhase::Connected) return;
  if (robot.connected()) {
    while (robot.available()) ingestStatus(static_cast<uint8_t>(robot.read()), now);
    return;
  }
  driveReady = false;
  lastStatusMs = 0;
  statusLength = 0;
  if (lastConnectAttemptMs != 0 && now - lastConnectAttemptMs < kReconnectIntervalMs) return;
  lastConnectAttemptMs = now;
  robot.setNoDelay(true);
  if (robot.connect(kRobotHost, kControlPort, 1000)) {
    sendAxes(0, 0);
  }
}

bool pressed(char key) { return M5Cardputer.Keyboard.isKeyPressed(key); }

void updateKeyboard(uint32_t now) {
  M5Cardputer.update();
  // The printed arrows use the semicolon, period, comma, and slash keys.
  const bool up = pressed(';');
  const bool down = pressed('.');
  const bool left = pressed(',');
  const bool right = pressed('/');
  const bool lower = pressed('[');
  const bool higher = pressed(']');
  const bool micLower = pressed('-');
  const bool micHigher = pressed('=');
  const bool video = pressed('v');
  const bool rescan = pressed('r');
  const bool enter = M5Cardputer.Keyboard.keysState().enter;

  if (wifiPhase != WifiPhase::Connected) {
    const bool navUp = up;
    const bool navDown = down;
    if (wifiPhase == WifiPhase::Choosing) {
      if (navUp && !navUpWasDown && selectedNetwork > 0) --selectedNetwork;
      if (navDown && !navDownWasDown && selectedNetwork + 1 < robotNetworkCount) ++selectedNetwork;
      if (enter && !enterWasDown) connectToSelectedRobot(now);
    } else if (wifiPhase == WifiPhase::Failed && enter && !enterWasDown) {
      WiFi.begin(selectedSsid);
      wifiConnectStartedMs = now;
      wifiPhase = WifiPhase::Connecting;
    }
    if (rescan && !rescanWasDown) beginWifiScan();
    navUpWasDown = navUp;
    navDownWasDown = navDown;
    rescanWasDown = rescan;
    enterWasDown = enter;
    driveReady = false;
    throttle = 0;
    steering = 0;
    return;
  }
  if (rescan && !rescanWasDown) {
    beginWifiScan();
    rescanWasDown = rescan;
    return;
  }
  rescanWasDown = rescan;

  if (video && !videoWasDown) toggleVideo();
  videoWasDown = video;

  if (micLower && !micLowerWasDown && micOutputPercent.load() > 10) {
    micOutputPercent.store(micOutputPercent.load() - 10);
  }
  if (micHigher && !micHigherWasDown && micOutputPercent.load() < 100) {
    micOutputPercent.store(micOutputPercent.load() + 10);
  }
  micLowerWasDown = micLower;
  micHigherWasDown = micHigher;

  for (size_t i = 0; i < sizeof kSpeedPresets / sizeof kSpeedPresets[0]; ++i) {
    if (pressed(static_cast<char>('1' + i))) {
      strength = kSpeedPresets[i];
      break;
    }
  }
  if (lower && !leftBracketWasDown && strength > 1) --strength;
  if (higher && !rightBracketWasDown && strength < 31) ++strength;
  displayedSpeed.store(strength);
  leftBracketWasDown = lower;
  rightBracketWasDown = higher;

  enterWasDown = enter;

  if (!robot.connected() || !statusFresh(now)) driveReady = false;
  if (!up && !down && !left && !right &&
      robot.connected() && statusFresh(now)) driveReady = true;
  const bool moving = driveReady;
  const int nextThrottle = moving ? (static_cast<int>(up) - static_cast<int>(down)) * strength : 0;
  const int nextSteering = moving ? (static_cast<int>(right) - static_cast<int>(left)) * strength : 0;
  if (nextThrottle != throttle || nextSteering != steering) {
    throttle = nextThrottle;
    steering = nextSteering;
    sendAxes(throttle, steering);
  }
}

void updateMicrophone() {
  const bool held = M5Cardputer.BtnA.isPressed();
  micButtonHeld.store(held);
  if (!held) micError = MicError::None;
  if (!held || WiFi.status() != WL_CONNECTED) {
    stopMicrophone();
    // Discard old samples so the next press starts with live audio.
    if (micReady) {
      for (int i = 0; i < 4; ++i) {
        size_t discarded = 0;
        i2s_channel_read(micChannel, micReadSamples, sizeof micReadSamples, &discarded, 0);
        voiceFilter.observeAmbient(micReadSamples, discarded / sizeof(int16_t));
        if (discarded == 0) break;
      }
    }
    return;
  }
  if (!talking && micError == MicError::None) {
    if (!micReady) micReady = beginMicrophone();
    if (!micReady) {
      micError = MicError::Start;
      return;
    }
    if (!audio.begin(kAudioPort)) {
      micError = MicError::Socket;
      return;
    }
    talking = true;
    micRunning.store(true);
    audioPacketsSent = 0;
    micCaptureCount = 0;
  }
  if (!talking) return;
  for (int i = 0; i < 3 && micCaptureCount < kMicCaptureSamples; ++i) {
    size_t bytesRead = 0;
    const size_t remaining = kMicCaptureSamples - micCaptureCount;
    const size_t toRead = remaining < kMicReadSamples ? remaining : kMicReadSamples;
    const esp_err_t result = i2s_channel_read(
        micChannel, micReadSamples, toRead * sizeof(int16_t), &bytesRead, 0);
    if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
      stopMicrophone();
      micError = MicError::Record;
      return;
    }
    const size_t samplesRead = bytesRead / sizeof(int16_t);
    memcpy(micCaptureSamples + micCaptureCount, micReadSamples, samplesRead * sizeof(int16_t));
    micCaptureCount += samplesRead;
    if (samplesRead == 0) break;
  }
  if (micCaptureCount < kMicCaptureSamples) return;
  micCaptureCount = 0;
  // The button or Wi-Fi may have disconnected while the buffer was filling.
  if (!M5Cardputer.BtnA.isPressed() || WiFi.status() != WL_CONNECTED) {
    stopMicrophone();
    return;
  }
  uint16_t peak = 0;
  uint16_t range = 0;
  voiceFilter.process(micCaptureSamples, audioSamples, kAudioSamples, peak, range,
                      micOutputPercent.load());
  audioPeak = peak;
  audioRange = range;
  if (!audio.beginPacket(kRobotHost, kAudioPort) ||
      audio.write(reinterpret_cast<const uint8_t *>(audioSamples), sizeof audioSamples) != sizeof audioSamples ||
      !audio.endPacket()) {
    stopMicrophone();
    micError = MicError::Send;
  } else {
    ++audioPacketsSent;
  }
}

void showCameraMessage(const char* message) {
  auto& display = M5Cardputer.Display;
  display.fillScreen(BLACK);
  cameraNeedsClear = true;
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(6, 35);
  display.println(message);
  display.setCursor(6, 119);
  display.print("V back  G0 hold to talk");
}

void drawCameraFrame(uint8_t* frame, size_t size, uint16_t fps) {
  const size_t jpegSize = claptrap::restoreCameraFrame(frame, size);
  uint16_t width = 0, height = 0;
  if (!jpegSize || !claptrap::jpegDimensions(frame, jpegSize, width, height)) return;
  float scale = 240.0f / width;
  const float heightScale = 119.0f / height;
  if (heightScale < scale) scale = heightScale;
  if (scale > 1.0f) scale = 1.0f;
  const int x = (240 - static_cast<int>(width * scale)) / 2;
  const int y = (119 - static_cast<int>(height * scale)) / 2;
  auto& display = M5Cardputer.Display;
  // Keep the last frame visible until the next one is ready. Clear the margins
  // only when the image size or position changes.
  static int previousX = -1, previousY = -1;
  static int previousWidth = -1, previousHeight = -1;
  const int drawnWidth = static_cast<int>(width * scale);
  const int drawnHeight = static_cast<int>(height * scale);
  if (cameraNeedsClear || x != previousX || y != previousY || drawnWidth != previousWidth ||
      drawnHeight != previousHeight) {
    display.fillRect(0, 0, 240, 119, BLACK);
    cameraNeedsClear = false;
    previousX = x;
    previousY = y;
    previousWidth = drawnWidth;
    previousHeight = drawnHeight;
  }
  display.drawJpg(frame, jpegSize, x, y, 240, 119, 0, 0, scale, scale);
  display.fillRect(0, 119, 240, 16, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(6, 122);
  if (!micButtonHeld.load()) {
    display.printf("V %u fps Spd:%d Vol:%u%%", fps, displayedSpeed.load(),
                   static_cast<unsigned>(micOutputPercent.load()));
  } else if (micRunning.load()) {
    display.printf("V %u fps Vol:%u%% Mic:%u/%u", fps,
                   static_cast<unsigned>(micOutputPercent.load()),
                   static_cast<unsigned>(audioPeak.load()),
                   static_cast<unsigned>(audioRange.load()));
  } else {
    display.printf("V %u fps Vol:%u%% G0:DOWN", fps,
                   static_cast<unsigned>(micOutputPercent.load()));
  }
}

bool readHttpLine(WiFiClient& camera, char* line, size_t capacity, uint32_t startedMs) {
  size_t length = 0;
  while (videoRequested.load() && millis() - startedMs < 3000) {
    if (!camera.available()) {
      if (!camera.connected()) return false;
      vTaskDelay(1);
      continue;
    }
    const int next = camera.read();
    if (next == '\n') {
      line[length] = '\0';
      return true;
    }
    if (next == '\r') continue;
    if (length + 1 >= capacity) return false;
    line[length++] = static_cast<char>(next);
  }
  return false;
}

void cameraTask(void*) {
  uint8_t* frame = static_cast<uint8_t*>(malloc(kCameraFrameCapacity));
  if (!frame) {
    showCameraMessage("Video memory unavailable");
    vTaskDelay(pdMS_TO_TICKS(1500));
    videoRequested.store(false);
    videoRunning.store(false);
    vTaskDelete(nullptr);
    return;
  }
  showCameraMessage("Video connecting...");
  while (videoRequested.load()) {
    if (WiFi.status() != WL_CONNECTED) {
      showCameraMessage("Waiting for robot Wi-Fi");
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    WiFiClient camera;
    if (!camera.connect(kRobotHost, kCameraPort, 1000)) {
      showCameraMessage("Camera reconnecting...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    camera.print("GET / HTTP/1.1\r\nHost: ");
    camera.print(kRobotHost);
    camera.print("\r\nAccept: multipart/x-mixed-replace, image/jpeg\r\nConnection: keep-alive\r\n\r\n");
    char line[256];
    const uint32_t headerStartedMs = millis();
    bool validResponse = readHttpLine(camera, line, sizeof line, headerStartedMs) &&
                         (strncmp(line, "HTTP/1.1 200", 12) == 0 ||
                          strncmp(line, "HTTP/1.0 200", 12) == 0);
    bool headersEnded = false;
    for (int i = 0; validResponse && i < 32; ++i) {
      if (!readHttpLine(camera, line, sizeof line, headerStartedMs)) break;
      if (line[0] == '\0') {
        headersEnded = true;
        break;
      }
    }
    if (!validResponse || !headersEnded) {
      camera.stop();
      showCameraMessage("Camera HTTP unavailable");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    showCameraMessage("Waiting for video frames");
    claptrap::CameraFrameParser parser(frame, kCameraFrameCapacity);
    uint32_t lastByteMs = millis();
    uint32_t fpsWindowMs = millis();
    uint16_t framesThisWindow = 0;
    uint16_t fps = 0;
    uint8_t input[kCameraReadChunk];
    while (videoRequested.load() && WiFi.status() == WL_CONNECTED && camera.connected()) {
      const int available = camera.available();
      if (available > 0) {
        const size_t request = static_cast<size_t>(available) < sizeof input ?
                               static_cast<size_t>(available) : sizeof input;
        const int received = camera.read(input, request);
        if (received <= 0) break;
        lastByteMs = millis();
        for (int i = 0; i < received && videoRequested.load(); ++i) {
          size_t frameSize = 0;
          if (parser.feed(input[i], frameSize)) {
            const uint32_t now = millis();
            if (now - fpsWindowMs >= 1000) {
              fps = static_cast<uint16_t>(framesThisWindow * 1000u / (now - fpsWindowMs));
              framesThisWindow = 0;
              fpsWindowMs = now;
            }
            drawCameraFrame(frame, frameSize, fps);
            ++framesThisWindow;
          }
          if (parser.takeOverflow()) showCameraMessage("Video frame too large");
        }
      }
      if (millis() - lastByteMs > 3000) break;
      if (available <= 0) vTaskDelay(1);
    }
    camera.stop();
    if (videoRequested.load()) {
      showCameraMessage("Camera reconnecting...");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  free(frame);
  videoRunning.store(false);
  vTaskDelete(nullptr);
}

void drawStatusLine(size_t index, int y, const char* value, bool force) {
  static char previous[7][64] = {};
  if (!force && strcmp(previous[index], value) == 0) return;
  auto& display = M5Cardputer.Display;
  display.fillRect(0, y, 240, y >= 126 ? 9 : 15, BLACK);
  display.setCursor(6, y);
  display.print(value);
  strncpy(previous[index], value, sizeof previous[index] - 1);
  previous[index][sizeof previous[index] - 1] = '\0';
}

void draw(uint32_t now) {
  if (videoRequested.load() || videoRunning.load()) return;
  if (now - lastDrawMs < 200) return;
  lastDrawMs = now;
  auto &display = M5Cardputer.Display;
  static WifiPhase drawnPhase = WifiPhase::Scanning;
  static size_t drawnNetwork = 0;
  static size_t drawnNetworkCount = 0;
  const bool wifiChanged = wifiPhase != drawnPhase ||
                           selectedNetwork != drawnNetwork ||
                           robotNetworkCount != drawnNetworkCount;
  const bool fullRedraw = screenNeedsClear || wifiChanged;
  if (fullRedraw) {
    display.fillScreen(BLACK);
    display.setTextColor(ORANGE, BLACK);
    display.setTextSize(2);
    display.setCursor(6, 5);
    display.print("CLAPTRAP 0.4");
    screenNeedsClear = false;
    drawnPhase = wifiPhase;
    drawnNetwork = selectedNetwork;
    drawnNetworkCount = robotNetworkCount;
  }
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  if (wifiPhase != WifiPhase::Connected) {
    if (!fullRedraw) return;
    display.setCursor(6, 31);
    if (wifiPhase == WifiPhase::Scanning) {
      display.println("Scanning for robot Wi-Fi...");
    } else if (wifiPhase == WifiPhase::Choosing) {
      display.println("Select a CL4P-TP network:");
      const size_t first = (selectedNetwork / 4) * 4;
      for (size_t i = first; i < robotNetworkCount && i < first + 4; ++i) {
        display.setCursor(6, 48 + static_cast<int>(i - first) * 16);
        display.printf("%c %.25s%s", i == selectedNetwork ? '>' : ' ',
                       robotNetworks[i].ssid, robotNetworks[i].open ? "" : " [locked]");
      }
      display.setCursor(6, 116);
      display.print(robotNetworks[selectedNetwork].open ? "UP/DOWN choose  ENTER connect" : "Password-protected network");
    } else if (wifiPhase == WifiPhase::Connecting) {
      display.println("Connecting to:");
      display.setCursor(6, 51);
      display.println(selectedSsid);
    } else if (wifiPhase == WifiPhase::NoneFound) {
      display.println("No CL4P-TP Wi-Fi found");
      display.setCursor(6, 51);
      display.println("Turn on the robot and retry.");
    } else if (wifiPhase == WifiPhase::Failed) {
      display.println("Could not join robot Wi-Fi");
      display.setCursor(6, 51);
      display.println(selectedSsid);
      display.setCursor(6, 73);
      display.println("ENTER retry");
    }
    display.setCursor(6, 126);
    display.print("R rescan");
    return;
  }
  char line[64];
  snprintf(line, sizeof line, "Wi-Fi: %s", selectedSsid);
  drawStatusLine(0, 32, line, fullRedraw);
  snprintf(line, sizeof line, "Robot: %s", robot.connected() ?
           (statusFresh(now) ? "online" : "waiting for status") : "connecting");
  drawStatusLine(1, 48, line, fullRedraw);
  snprintf(line, sizeof line, "Movement: %s",
           !robot.connected() || !statusFresh(now) ? "waiting for robot" :
           driveReady ? "ready" : "release arrow keys");
  drawStatusLine(2, 64, line, fullRedraw);
  snprintf(line, sizeof line, "Speed: %d/31  T:%d S:%d", strength, throttle, steering);
  drawStatusLine(3, 80, line, fullRedraw);
  if (micError != MicError::None) {
    const char* reason = micError == MicError::Socket ? "UDP socket" :
                         micError == MicError::Start ? "mic start" :
                         micError == MicError::Record ? "record" : "UDP send";
    snprintf(line, sizeof line, "Mic: ERROR %s", reason);
  } else if (talking) {
    snprintf(line, sizeof line, "Mic:G0 DOWN p:%u r:%u tx:%lu",
             static_cast<unsigned>(audioPeak.load()),
             static_cast<unsigned>(audioRange.load()),
             static_cast<unsigned long>(audioPacketsSent));
  } else if (WiFi.status() != WL_CONNECTED) {
    snprintf(line, sizeof line, "Mic: G0 %s, Wi-Fi needed",
             micButtonHeld.load() ? "DOWN" : "UP");
  } else {
    snprintf(line, sizeof line, "Mic: G0 %s, hold to talk",
             micButtonHeld.load() ? "DOWN" : "UP");
  }
  drawStatusLine(4, 96, line, fullRedraw);
  if (videoStartFailed) {
    drawStatusLine(5, 111, "Video unavailable", fullRedraw);
  } else {
    snprintf(line, sizeof line, "Mic output: %u%%  - / = adjust",
             static_cast<unsigned>(micOutputPercent.load()));
    drawStatusLine(5, 111, line, fullRedraw);
  }
  drawStatusLine(6, 126, "G0 talk  V camera  1-5 speed", fullRedraw);
}
}  // namespace

void setup() {
  auto config = M5.config();
  // The sketch sets up the microphone itself, so disable the library's audio setup.
  config.internal_mic = false;
  config.internal_spk = false;
  M5Cardputer.begin(config, true);
  M5Cardputer.Display.setRotation(1);
  micReady = beginMicrophone();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
}

void loop() {
  const uint32_t now = millis();
  updateKeyboard(now);
  updateWifi(now);
  updateConnection(now);
  updateMicrophone();
  if (robot.connected() && now - lastSendMs >= kSendIntervalMs) {
    if (!statusFresh(now)) {
      driveReady = false;
      throttle = 0;
      steering = 0;
    }
    sendAxes(throttle, steering);
  }
  draw(now);
  delay(talking ? 1 : 10);
}
