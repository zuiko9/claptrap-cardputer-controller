#include "../ClaptrapCardputer/protocol.h"

#include <assert.h>
#include <string.h>

int main() {
  uint8_t packet[claptrap::kPacketSize];
  claptrap::buildPacket(0, 0, packet);
  const uint8_t neutral[] = {0xaa, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0x82, 0xbb};
  assert(memcmp(packet, neutral, sizeof neutral) == 0);
  claptrap::buildPacket(31, 0, packet);
  assert(packet[9] == 0x1f && packet[20] == 0xa1);
  claptrap::buildPacket(-31, 0, packet);
  assert(packet[9] == 0x9f && packet[20] == 0x21);
  claptrap::buildPacket(0, 31, packet);
  assert(packet[7] == 0x1f && packet[20] == 0x63);
  claptrap::buildPacket(0, -31, packet);
  assert(packet[7] == 0x9f && packet[20] == 0xe3);
  return 0;
}
