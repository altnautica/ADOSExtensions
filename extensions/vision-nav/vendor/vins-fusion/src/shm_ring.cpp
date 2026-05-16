#include "shm_ring.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace ados::vision_nav {

namespace {
constexpr size_t kSlotCount = 8;
constexpr size_t kMaxPayloadBytes = 4 * 1024 * 1024;  // up to 4 MiB per frame
constexpr size_t kSlotSize = sizeof(FrameSlotHeader) + kMaxPayloadBytes;
constexpr size_t kRingSize = kSlotCount * kSlotSize;
}  // namespace

ShmRing::ShmRing(const std::string& path) : path_(path) {
  int fd = ::shm_open(path_.c_str(), O_RDONLY, 0);
  if (fd < 0) {
    throw std::runtime_error("shm_open failed: " + path_);
  }
  struct stat st {};
  if (::fstat(fd, &st) < 0) {
    ::close(fd);
    throw std::runtime_error("fstat failed");
  }
  mapped_size_ = static_cast<size_t>(st.st_size);
  base_ = ::mmap(nullptr, mapped_size_, PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (base_ == MAP_FAILED) {
    base_ = nullptr;
    throw std::runtime_error("mmap failed");
  }
}

ShmRing::~ShmRing() {
  if (base_ != nullptr) {
    ::munmap(base_, mapped_size_);
  }
}

bool ShmRing::next(FrameSlotHeader& header, const unsigned char*& data) {
  // Walk the slots looking for one with a sequence greater than the
  // last we returned. The plugin's Python side increments sequences
  // monotonically across slots so the wrap-around is captured by
  // comparing on the uint64.
  unsigned char* base = static_cast<unsigned char*>(base_);
  uint64_t best_seq = last_sequence_;
  ssize_t best_slot = -1;
  for (size_t i = 0; i < kSlotCount; i++) {
    const auto* h = reinterpret_cast<const FrameSlotHeader*>(
        base + i * kSlotSize);
    if (h->sequence > best_seq) {
      best_seq = h->sequence;
      best_slot = static_cast<ssize_t>(i);
    }
  }
  if (best_slot < 0) return false;
  const auto* h = reinterpret_cast<const FrameSlotHeader*>(
      base + best_slot * kSlotSize);
  header = *h;
  data = base + best_slot * kSlotSize + sizeof(FrameSlotHeader);
  last_sequence_ = h->sequence;
  return true;
}

}  // namespace ados::vision_nav
