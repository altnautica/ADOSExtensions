// SPDX-License-Identifier: BSD-3-Clause
//
// Input stream protocol of basalt-ados (see README.md).
//
// The capture service binds a Unix stream socket and basalt-ados connects to
// it. Every message is a 5-byte header followed by `len` payload bytes, all
// little-endian:
//
//   u8  type
//   u32 len
//   u8  payload[len]
//
//   type 0 HELLO   payload: u32 version (= kProtocolVersion); first message.
//   type 1 IMU     payload: u64 t_ns, f64 gyro[3] rad/s, f64 accel[3] m/s^2,
//                  IMU sensor frame, len = 56.
//   type 2 STEREO  payload: u64 t_ns, u32 width, u32 height,
//                  u8 left[width*height], u8 right[width*height] (Gray8,
//                  row-major, no padding), len = 16 + 2*width*height.
//
// Times are host CLOCK_MONOTONIC nanoseconds. Unknown types are skipped.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ados {

constexpr uint32_t kProtocolVersion = 1;

constexpr uint8_t kMsgHello = 0;
constexpr uint8_t kMsgImu = 1;
constexpr uint8_t kMsgStereo = 2;

constexpr size_t kHeaderLen = 5;
constexpr uint32_t kHelloLen = 4;
constexpr uint32_t kImuLen = 56;
constexpr uint32_t kStereoHeaderLen = 16;
// Larger than any stereo pair the rig produces (2 x 4096 x 4096 Gray8).
constexpr uint32_t kMaxPayloadLen = 2u * 4096u * 4096u + kStereoHeaderLen;

inline uint32_t read_u32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 |
         uint32_t(p[3]) << 24;
}

inline uint64_t read_u64(const uint8_t* p) {
  return uint64_t(read_u32(p)) | uint64_t(read_u32(p + 4)) << 32;
}

inline double read_f64(const uint8_t* p) {
  const uint64_t bits = read_u64(p);
  double v;
  std::memcpy(&v, &bits, sizeof v);
  return v;
}

struct ImuMsg {
  int64_t t_ns;
  double gyro[3];
  double accel[3];
};

inline ImuMsg decode_imu(const uint8_t* p) {
  ImuMsg m;
  m.t_ns = int64_t(read_u64(p));
  for (int i = 0; i < 3; i++) m.gyro[i] = read_f64(p + 8 + 8 * i);
  for (int i = 0; i < 3; i++) m.accel[i] = read_f64(p + 32 + 8 * i);
  return m;
}

struct StereoHeader {
  int64_t t_ns;
  uint32_t width;
  uint32_t height;
};

inline StereoHeader decode_stereo_header(const uint8_t* p) {
  return StereoHeader{int64_t(read_u64(p)), read_u32(p + 8), read_u32(p + 12)};
}

}  // namespace ados
