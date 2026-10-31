#pragma once

#include <stddef.h>
#include <stdint.h>

namespace claptrap {

// The robot changes one byte in each camera image and replaces the JPEG ending
// with a length trailer. Restore those bytes before decoding the image.
inline size_t restoreCameraFrame(uint8_t* frame, size_t size) {
  if (size < 263) return size;
  const size_t imageLength = (static_cast<size_t>(frame[size - 6]) << 24) |
                             (static_cast<size_t>(frame[size - 5]) << 16) |
                             (static_cast<size_t>(frame[size - 4]) << 8) |
                             static_cast<size_t>(frame[size - 3]);
  if (imageLength != size - 4) return size;
  const size_t offset = imageLength / 2 +
                        (static_cast<size_t>(frame[size - 149]) + frame[size - 263]) % 10;
  if (offset >= imageLength) return 0;
  frame[size - 6] = frame[size - 2];
  frame[size - 5] = frame[size - 1];
  frame[size - 4] = 0;
  frame[offset] ^= 0xff;
  return imageLength;
}

// Read the image dimensions from the JPEG header without decoding the image.
inline bool jpegDimensions(const uint8_t* data, size_t size, uint16_t& width, uint16_t& height) {
  if (size < 4 || data[0] != 0xff || data[1] != 0xd8) return false;
  size_t pos = 2;
  while (pos + 4 <= size) {
    if (data[pos++] != 0xff) return false;
    while (pos < size && data[pos] == 0xff) ++pos;
    if (pos >= size) return false;
    const uint8_t marker = data[pos++];
    if (marker == 0xd9 || marker == 0xda) return false;
    if (marker == 0x01 || (marker >= 0xd0 && marker <= 0xd7)) continue;
    if (pos + 2 > size) return false;
    const size_t segmentLength = (static_cast<size_t>(data[pos]) << 8) | data[pos + 1];
    if (segmentLength < 2 || segmentLength > size - pos) return false;
    if ((marker >= 0xc0 && marker <= 0xc3) && segmentLength >= 7) {
      height = static_cast<uint16_t>((data[pos + 3] << 8) | data[pos + 4]);
      width = static_cast<uint16_t>((data[pos + 5] << 8) | data[pos + 6]);
      return width != 0 && height != 0;
    }
    pos += segmentLength;
  }
  return false;
}

class CameraFrameParser {
 public:
  CameraFrameParser(uint8_t* buffer, size_t capacity)
      : buffer_(buffer), capacity_(capacity) {}

  bool feed(uint8_t byte, size_t& frameSize) {
    if (length_ == 0) {
      if (previous_ == 0xff && byte == 0xd8 && capacity_ >= 2) {
        buffer_[0] = 0xff;
        buffer_[1] = 0xd8;
        length_ = 2;
      }
      previous_ = byte;
      return false;
    }
    if (length_ >= capacity_) {
      length_ = 0;
      overflowed_ = true;
      previous_ = byte;
      return false;
    }
    buffer_[length_++] = byte;
    const bool complete = previous_ == 0xff && byte == 0xd9;
    previous_ = byte;
    if (!complete) return false;
    frameSize = length_;
    length_ = 0;
    return true;
  }

  bool takeOverflow() {
    const bool result = overflowed_;
    overflowed_ = false;
    return result;
  }

 private:
  uint8_t* buffer_;
  size_t capacity_;
  size_t length_ = 0;
  uint8_t previous_ = 0;
  bool overflowed_ = false;
};

}  // namespace claptrap
