// Shared-memory frame ring adapter.
//
// Matches the Python-side ``ShmFrameRing`` at
// ../../agent/src/altnautica_vision_nav/shim/ipc.py: a fixed-capacity
// ring of frame slots, each with a header describing the frame's
// width, height, stride, format, and a monotonic sequence number.
// The vendor binary mmaps the region and reads slots as the Python
// side advances the write index.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace ados::vision_nav {

struct FrameSlotHeader {
  uint64_t sequence;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t format;       // V4L2 fourcc
  uint64_t timestamp_ns;
  uint32_t payload_size;
};

class ShmRing {
 public:
  explicit ShmRing(const std::string& path);
  ~ShmRing();

  // Read the next available slot. Returns ``false`` when no new
  // slot is ready since the last call.
  bool next(FrameSlotHeader& header, const unsigned char*& data);

 private:
  std::string path_;
  void* base_ = nullptr;
  size_t mapped_size_ = 0;
  uint64_t last_sequence_ = 0;
};

}  // namespace ados::vision_nav
