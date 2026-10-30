#pragma once

#include <stddef.h>
#include <stdint.h>

namespace claptrap {

// Convert microphone samples to the 11,025 Hz format the robot expects. Keep
// the gain and silence gate steady as one audio packet leads into the next.
class VoiceFilter {
 public:
  void release() {
    holdPackets_ = 0;
    gateQ8_ = 0;
  }

  void observeAmbient(const int16_t* samples, size_t count) {
    uint32_t sum = 0;
    size_t pairs = 0;
    for (size_t i = 0; i + 1 < count; i += 2) {
      sum += magnitude(filterPair(samples[i], samples[i + 1]));
      ++pairs;
    }
    if (!pairs) return;
    const uint32_t level = sum / pairs;
    // Learn the background level while G0 is released. Adjust more quickly
    // when the room gets quieter.
    if (level < noiseFloor_) noiseFloor_ = (noiseFloor_ * 3 + level) / 4;
    else noiseFloor_ = (noiseFloor_ * 7 + level) / 8;
  }

  void process(const int16_t* input, int16_t* output, size_t count,
               uint16_t& peak, uint16_t& range, uint8_t outputPercent = 100) {
    if (!count) {
      peak = 0;
      range = 0;
      return;
    }
    uint32_t sum = 0;
    int32_t low = 32767;
    int32_t high = -32768;
    peak = 0;
    for (size_t i = 0; i < count; ++i) {
      const int32_t sample = filterPair(input[i * 2], input[i * 2 + 1]);
      output[i] = static_cast<int16_t>(sample);
      if (sample < low) low = sample;
      if (sample > high) high = sample;
      const uint32_t level = magnitude(sample);
      sum += level;
      if (level > peak) peak = static_cast<uint16_t>(level);
    }
    range = static_cast<uint16_t>(high - low);
    const uint32_t meanLevel = count ? sum / count : 0;
    const uint32_t threshold = noiseFloor_ * 2 > 80 ? noiseFloor_ * 2 : 80;
    if (meanLevel > threshold) {
      holdPackets_ = 3;  // Keep the gate open briefly between syllables.
      uint32_t desired = peak ? (12000u * 256u) / peak : 256;
      if (desired > 1024) desired = 1024;
      if (desired < 128) desired = 128;
      if (!gainReady_) {
        gainQ8_ = desired;
        gainReady_ = true;
      } else {
        gainQ8_ = (gainQ8_ * 7 + desired) / 8;
      }
    } else if (holdPackets_) {
      --holdPackets_;
    }
    const uint16_t gateTarget = holdPackets_ ? 256 : 0;
    if (outputPercent > 100) outputPercent = 100;
    for (size_t i = 0; i < count; ++i) {
      if (gateQ8_ < gateTarget) {
        gateQ8_ = gateQ8_ + 4 > gateTarget ? gateTarget : gateQ8_ + 4;
      } else if (gateQ8_ > gateTarget) {
        gateQ8_ = gateQ8_ < 2 ? 0 : gateQ8_ - 2;
      }
      int32_t sample = static_cast<int32_t>(output[i]) * static_cast<int32_t>(gainQ8_) / 256;
      sample = sample * gateQ8_ / 256;
      sample = sample * outputPercent / 100;
      if (sample > 32767) sample = 32767;
      if (sample < -32768) sample = -32768;
      output[i] = static_cast<int16_t>(sample);
    }
  }

 private:
  static uint32_t magnitude(int32_t value) {
    return static_cast<uint32_t>(value < 0 ? -value : value);
  }

  int32_t filterPair(int16_t first, int16_t second) {
    // Average each pair of samples before halving the sample rate.
    const int32_t input = (static_cast<int32_t>(first) + second) / 2;
    // Remove steady offsets and low rumble from the microphone signal.
    const int32_t result = input - previousInput_ +
                           static_cast<int32_t>((static_cast<int64_t>(previousOutput_) * 31280) / 32768);
    previousInput_ = input;
    previousOutput_ = result;
    if (result > 32767) return 32767;
    if (result < -32768) return -32768;
    return result;
  }

  int32_t previousInput_ = 0;
  int32_t previousOutput_ = 0;
  uint32_t noiseFloor_ = 64;
  uint32_t gainQ8_ = 256;
  uint16_t gateQ8_ = 0;
  uint8_t holdPackets_ = 0;
  bool gainReady_ = false;
};

}  // namespace claptrap
