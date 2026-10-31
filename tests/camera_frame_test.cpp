#include "../ClaptrapCardputer/camera_frame.h"

#include <cstdlib>
#include <string.h>

void check(bool condition) {
  if (!condition) std::abort();
}

int main() {
  uint8_t frame[300];
  for (size_t i = 0; i < sizeof frame; ++i) frame[i] = static_cast<uint8_t>((i * 37 + 11) % 256);
  frame[0] = 0xff;
  frame[1] = 0xd8;
  frame[294] = 0;
  frame[295] = 0;
  frame[296] = 1;
  frame[297] = 0x28;  // 296 image bytes.
  frame[298] = 0xff;
  frame[299] = 0xd9;
  uint8_t expected[300];
  memcpy(expected, frame, sizeof frame);
  const size_t correction = 296 / 2 + (frame[151] + frame[37]) % 10;
  expected[correction] ^= 0xff;
  expected[294] = 0xff;
  expected[295] = 0xd9;
  check(claptrap::restoreCameraFrame(frame, sizeof frame) == 296);
  check(memcmp(frame, expected, 296) == 0);

  uint8_t buffer[8] = {};
  claptrap::CameraFrameParser parser(buffer, sizeof buffer);
  size_t length = 0;
  const uint8_t stream[] = {1, 0xff, 0xd8, 2, 3, 0xff, 0xd9};
  for (size_t i = 0; i < sizeof stream - 1; ++i) check(!parser.feed(stream[i], length));
  check(parser.feed(stream[sizeof stream - 1], length));
  check(length == 6 && buffer[0] == 0xff && buffer[5] == 0xd9);

  const uint8_t oversized[] = {0xff, 0xd8, 1, 2, 3, 4, 5, 6, 7};
  for (uint8_t byte : oversized) check(!parser.feed(byte, length));
  check(parser.takeOverflow());
  check(!parser.takeOverflow());
  for (size_t i = 0; i < sizeof stream - 1; ++i) check(!parser.feed(stream[i], length));
  check(parser.feed(stream[sizeof stream - 1], length));
  check(length == 6);

  const uint8_t jpeg[] = {0xff, 0xd8, 0xff, 0xc0, 0, 7, 8, 0, 135, 0, 240};
  uint16_t width = 0, height = 0;
  check(claptrap::jpegDimensions(jpeg, sizeof jpeg, width, height));
  check(width == 240 && height == 135);
  return 0;
}
