#include "../ClaptrapCardputer/voice_filter.h"

#include <assert.h>
#include <stdint.h>

namespace {
constexpr size_t kInputSamples = 1280;
constexpr size_t kOutputSamples = kInputSamples / 2;

uint16_t maximum(const int16_t* samples) {
  uint16_t peak = 0;
  for (size_t i = 0; i < kOutputSamples; ++i) {
    const int32_t sample = samples[i];
    const uint16_t magnitude = static_cast<uint16_t>(sample < 0 ? -sample : sample);
    if (magnitude > peak) peak = magnitude;
  }
  return peak;
}
}  // namespace

int main() {
  claptrap::VoiceFilter filter;
  int16_t ambient[256];
  for (size_t i = 0; i < 256; ++i) ambient[i] = (i / 16) % 2 ? 100 : -100;
  for (int i = 0; i < 20; ++i) filter.observeAmbient(ambient, 256);

  int16_t input[kInputSamples];
  int16_t output[kOutputSamples];
  for (size_t i = 0; i < kInputSamples; ++i) input[i] = (i / 16) % 2 ? 100 : -100;
  uint16_t peak = 0, range = 0;
  filter.process(input, output, kOutputSamples, peak, range);
  assert(peak > 0 && range > 0);
  assert(maximum(output) == 0);

  for (size_t i = 0; i < kInputSamples; ++i) input[i] = (i / 24) % 2 ? 3000 : -3000;
  filter.process(input, output, kOutputSamples, peak, range);
  assert(peak > 1000 && maximum(output) > 1000);

  // Keep the gate open through a brief pause, then close it during silence.
  for (size_t i = 0; i < kInputSamples; ++i) input[i] = (i / 16) % 2 ? 100 : -100;
  filter.process(input, output, kOutputSamples, peak, range);
  assert(maximum(output) > 0);
  for (int i = 0; i < 6; ++i) filter.process(input, output, kOutputSamples, peak, range);
  assert(maximum(output) == 0);

  // A steady microphone offset should fade away.
  for (size_t i = 0; i < kInputSamples; ++i) input[i] = 1200;
  for (int i = 0; i < 4; ++i) filter.process(input, output, kOutputSamples, peak, range);
  assert(peak < 10);

  // Lower the volume of the sent audio without changing the captured signal.
  claptrap::VoiceFilter fullVolume;
  claptrap::VoiceFilter lowVolume;
  for (int i = 0; i < 20; ++i) {
    fullVolume.observeAmbient(ambient, 256);
    lowVolume.observeAmbient(ambient, 256);
  }
  for (size_t i = 0; i < kInputSamples; ++i) input[i] = (i / 24) % 2 ? 3000 : -3000;
  int16_t fullOutput[kOutputSamples];
  int16_t lowOutput[kOutputSamples];
  fullVolume.process(input, fullOutput, kOutputSamples, peak, range, 100);
  lowVolume.process(input, lowOutput, kOutputSamples, peak, range, 40);
  assert(fullOutput[0] < 0 && lowOutput[0] < 0);  // Negative samples must stay negative.
  const uint32_t fullPeak = maximum(fullOutput);
  const uint32_t lowPeak = maximum(lowOutput);
  assert(fullPeak > 0);
  assert(lowPeak >= fullPeak * 35 / 100 && lowPeak <= fullPeak * 45 / 100);

  fullVolume.release();
  for (size_t i = 0; i < kInputSamples; ++i) input[i] = (i / 16) % 2 ? 100 : -100;
  fullVolume.process(input, fullOutput, kOutputSamples, peak, range);
  assert(maximum(fullOutput) == 0);  // Releasing G0 closes the gate.
  return 0;
}
