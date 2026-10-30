#pragma once

#include <stddef.h>
#include <stdint.h>

namespace claptrap {

constexpr size_t kPacketSize = 22;

inline uint8_t encodeAxis(int value) {
  if (value > 31) value = 31;
  if (value < -31) value = -31;
  return value < 0 ? static_cast<uint8_t>(0x80 | -value)
                   : static_cast<uint8_t>(value);
}

inline uint8_t checksum(const uint8_t packet[kPacketSize]) {
  int value = 0x7c;
  constexpr uint8_t add[] = {1, 2, 4, 7, 11, 16};
  constexpr uint8_t subtract[] = {3, 5, 6, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19};
  for (uint8_t index : add) value += packet[index];
  for (uint8_t index : subtract) value -= packet[index];
  return static_cast<uint8_t>(~value);
}

inline void buildPacket(int throttle, int steering, uint8_t packet[kPacketSize]) {
  for (size_t i = 0; i < kPacketSize; ++i) packet[i] = 0;
  packet[0] = 0xaa;
  packet[1] = 1;
  packet[7] = encodeAxis(steering);
  packet[9] = encodeAxis(throttle);
  packet[20] = checksum(packet);
  packet[21] = 0xbb;
}

}  // namespace claptrap
