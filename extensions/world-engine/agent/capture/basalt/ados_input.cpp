// SPDX-License-Identifier: BSD-3-Clause
//
// basalt-ados: Basalt stereo-inertial VIO fed from a Unix socket.
//
//   basalt-ados <socket> <calib.json> <config.json>
//
// Connects to <socket>, reads the framed IMU + stereo stream described in
// ados_input.h, runs Basalt frame-to-frame optical flow and the square-root
// keypoint VIO estimator (float), and writes JSON lines to stdout:
//
//   {"status":"initializing"|"tracking"|"lost"}   on every status change
//   {"t_ns":u64,"T_w_i":[16 row-major],"cov":null,"features":n}
//
// T_w_i is the IMU pose in Basalt's world frame: gravity-aligned (z up),
// origin and yaw where the estimator initialised. Logs go to stderr.
//
// Exit codes: 0 input closed, 2 bad arguments/calibration/input, 3 tracking
// lost or never established (the supervisor restarts the process, which
// re-initialises the estimator).

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <basalt/calibration/calibration.hpp>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include "ados_input.h"

namespace {

using Clock = std::chrono::steady_clock;

// Fewer tracked features than this for kLowFeatureFrames frames in a row is
// reported as lost.
constexpr size_t kMinFeatures = 10;
constexpr int kLowFeatureFrames = 15;
// No state for this long while frames still arrive is reported as lost.
constexpr auto kStateTimeout = std::chrono::milliseconds(1000);
// Lost for this long, or no first state this long after the first frame:
// exit 3 so the supervisor restarts (and so re-initialises) the estimator.
constexpr auto kLostExit = std::chrono::milliseconds(3000);
constexpr auto kInitExit = std::chrono::milliseconds(15000);
// The input thread reads IMU and stereo messages in arrival order from one
// socket. Basalt's estimator takes IMU samples only while it integrates up to
// a vision frame, and its stock IMU queue holds 300 samples (0.3 s at 1 kHz).
// IMU that arrives before the first stereo pair, or while optical flow lags
// by more than 0.3 s, would fill that queue and block the reader on push, so
// the next stereo pair is never read and the estimator never consumes: a
// deadlock. So IMU before the first stereo pair is held back in a pre-roll of
// at most kImuPreroll samples, and the queue holds kImuQueueCapacity (20 s at
// 1 kHz, above the optical-flow queue's 10 frames of lag).
//
// A frame reaches this process later than the IMU of its exposure (render,
// encode, network and decode on a simulator; seconds on a loaded host), so the
// first pair is usually older than the newest IMU. The pre-roll therefore
// spans kImuPreroll samples (10 s at 1 kHz), and the first pair starts the
// queue from kImuLeadNs before its own time: the estimator then integrates
// from the first frame on instead of finding every IMU sample after it.
//
// The same holds the other way round: the estimator waits for IMU up to each
// frame's time, and that IMU sits in the socket behind later frames. A reader
// blocked on a full optical-flow queue (10 frames, which fills whenever flow
// runs slower than the cameras) never reads it, so the estimator never
// finishes the frame that would drain the queue. Stereo pairs are therefore
// offered with try_push and dropped when the queue is full; the reader never
// blocks on vision.
constexpr size_t kImuPreroll = 10000;
constexpr int64_t kImuLeadNs = 100'000'000;
constexpr int kImuQueueCapacity = 20000;

std::mutex g_out_mutex;
// The JSON-lines stream. Process stdout is pointed at stderr at start-up, so
// anything Basalt itself prints lands in the log, never in this stream.
FILE* g_out = nullptr;

void claim_stdout() {
  std::fflush(stdout);
  const int fd = ::dup(STDOUT_FILENO);
  if (fd < 0 || ::dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
    std::fprintf(stderr, "basalt-ados: cannot redirect stdout\n");
    std::_Exit(2);
  }
  g_out = ::fdopen(fd, "w");
  if (!g_out) std::_Exit(2);
}

void emit(const std::string& line) {
  std::lock_guard<std::mutex> lock(g_out_mutex);
  std::fwrite(line.data(), 1, line.size(), g_out);
  std::fputc('\n', g_out);
  std::fflush(g_out);
}

[[noreturn]] void die(int code, const std::string& why) {
  std::fprintf(stderr, "basalt-ados: %s\n", why.c_str());
  if (g_out) std::fflush(g_out);
  std::fflush(stderr);
  std::_Exit(code);
}

bool read_exact(int fd, uint8_t* buf, size_t n) {
  while (n > 0) {
    const ssize_t r = ::read(fd, buf, n);
    if (r == 0) return false;
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    buf += r;
    n -= size_t(r);
  }
  return true;
}

int connect_unix(const std::string& path) {
  sockaddr_un addr{};
  if (path.size() >= sizeof(addr.sun_path)) die(2, "socket path too long");
  addr.sun_family = AF_UNIX;
  std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
  const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) die(2, "socket(): " + std::string(std::strerror(errno)));
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0) {
    die(2, "connect " + path + ": " + std::strerror(errno));
  }
  return fd;
}

void append_num(std::string& s, double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.17g", v);
  s += buf;
}

std::string state_line(int64_t t_ns, const Sophus::SE3d& T_w_i,
                       long features) {
  const Eigen::Matrix4d m = T_w_i.matrix();
  std::string s = "{\"t_ns\":" + std::to_string(t_ns) + ",\"T_w_i\":[";
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      if (r || c) s += ',';
      append_num(s, m(r, c));
    }
  }
  s += "],\"cov\":null,\"features\":";
  s += features < 0 ? "null" : std::to_string(features);
  s += '}';
  return s;
}

basalt::Calibration<double> load_calib(const std::string& path) {
  basalt::Calibration<double> calib;
  std::ifstream is(path, std::ios::binary);
  if (!is.is_open()) die(2, "cannot open calibration " + path);
  try {
    cereal::JSONInputArchive archive(is);
    archive(calib);
  } catch (const std::exception& e) {
    die(2, "bad calibration " + path + ": " + e.what());
  }
  if (calib.intrinsics.size() != 2 || calib.T_i_c.size() != 2 ||
      calib.resolution.size() != 2) {
    die(2, "calibration must hold exactly two cameras (left, right)");
  }
  return calib;
}

basalt::VioConfig load_config(const std::string& path) {
  basalt::VioConfig config;
  std::ifstream probe(path);
  if (!probe.is_open()) die(2, "cannot open config " + path);
  probe.close();
  try {
    config.load(path);
  } catch (const std::exception& e) {
    die(2, "bad config " + path + ": " + e.what());
  }
  return config;
}

// Shared between the input, flow-forwarding and output threads.
struct Shared {
  std::mutex mutex;
  std::map<int64_t, size_t> features_at;  // frame t_ns -> tracked features
  int low_feature_run = 0;
  std::atomic<int64_t> frames_in{0};
  std::atomic<int64_t> last_frame_ms{0};
  uint64_t frames_dropped = 0;  // input thread only
};

int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             Clock::now().time_since_epoch())
      .count();
}

basalt::ManagedImage<uint16_t>::Ptr to_basalt_image(const uint8_t* gray,
                                                    uint32_t w, uint32_t h) {
  basalt::ManagedImage<uint16_t>::Ptr img(
      new basalt::ManagedImage<uint16_t>(w, h));
  // Basalt works on 16-bit intensities; 8-bit data is scaled by 256 as its
  // dataset readers do.
  for (uint32_t y = 0; y < h; y++) {
    uint16_t* row = img->RowPtr(y);
    const uint8_t* src = gray + size_t(y) * w;
    for (uint32_t x = 0; x < w; x++) row[x] = uint16_t(src[x]) << 8;
  }
  return img;
}

void input_loop(int fd, const basalt::Calibration<double>& calib,
                basalt::OpticalFlowBase& flow, basalt::VioEstimatorBase& vio,
                Shared& shared) {
  std::vector<uint8_t> buf;
  uint8_t header[ados::kHeaderLen];
  bool hello = false;
  bool seen_stereo = false;
  std::deque<basalt::ImuData<double>::Ptr> preroll;
  int64_t last_imu_t_ns = 0;
  while (read_exact(fd, header, sizeof header)) {
    const uint8_t type = header[0];
    const uint32_t len = ados::read_u32(header + 1);
    if (len > ados::kMaxPayloadLen) die(2, "oversized message");
    buf.resize(len);
    if (!read_exact(fd, buf.data(), len)) break;
    if (!hello) {
      if (type != ados::kMsgHello || len != ados::kHelloLen ||
          ados::read_u32(buf.data()) != ados::kProtocolVersion) {
        die(2, "expected HELLO with protocol version 1");
      }
      hello = true;
      continue;
    }
    if (type == ados::kMsgImu) {
      if (len != ados::kImuLen) die(2, "bad IMU message length");
      const ados::ImuMsg m = ados::decode_imu(buf.data());
      basalt::ImuData<double>::Ptr d(new basalt::ImuData<double>);
      d->t_ns = m.t_ns;
      last_imu_t_ns = m.t_ns;
      d->gyro = Eigen::Vector3d(m.gyro[0], m.gyro[1], m.gyro[2]);
      d->accel = Eigen::Vector3d(m.accel[0], m.accel[1], m.accel[2]);
      if (!seen_stereo) {
        preroll.push_back(d);
        if (preroll.size() > kImuPreroll) preroll.pop_front();
        continue;
      }
      vio.imu_data_queue.push(d);
    } else if (type == ados::kMsgStereo) {
      if (len < ados::kStereoHeaderLen) die(2, "bad stereo message length");
      const ados::StereoHeader h = ados::decode_stereo_header(buf.data());
      const size_t plane = size_t(h.width) * h.height;
      if (len != ados::kStereoHeaderLen + 2 * plane) {
        die(2, "stereo message length does not match its size");
      }
      bool size_ok = true;
      for (int i = 0; i < 2; i++) {
        size_ok = size_ok && calib.resolution[i][0] == int(h.width) &&
                  calib.resolution[i][1] == int(h.height);
      }
      if (!size_ok) {
        std::fprintf(stderr,
                     "basalt-ados: frame %ux%u does not match calibration, "
                     "dropped\n",
                     h.width, h.height);
        continue;
      }
      basalt::OpticalFlowInput::Ptr in(new basalt::OpticalFlowInput);
      in->t_ns = h.t_ns;
      in->img_data.resize(2);
      const uint8_t* left = buf.data() + ados::kStereoHeaderLen;
      in->img_data[0].img = to_basalt_image(left, h.width, h.height);
      in->img_data[1].img = to_basalt_image(left + plane, h.width, h.height);
      shared.frames_in++;
      shared.last_frame_ms = now_ms();
      if (shared.frames_in % 200 == 1) {
        // Frame and IMU clocks must agree: a frame far ahead of the newest
        // IMU, or far behind it, points at a timestamp mismatch upstream.
        std::fprintf(stderr,
                     "basalt-ados: pair %lld t_ns %lld, newest IMU %lld, "
                     "frame minus IMU %.3f s\n",
                     static_cast<long long>(shared.frames_in.load()),
                     static_cast<long long>(h.t_ns),
                     static_cast<long long>(last_imu_t_ns),
                     (double(h.t_ns) - double(last_imu_t_ns)) * 1e-9);
      }
      if (!seen_stereo) {
        // The IMU from just before the first frame, so integration starts there.
        for (auto& p : preroll) {
          if (p->t_ns + kImuLeadNs >= int64_t(h.t_ns)) vio.imu_data_queue.push(p);
        }
        preroll.clear();
        seen_stereo = true;
      }
      if (!flow.input_queue.try_push(in)) {
        // Optical flow is behind: drop this pair rather than block the reader.
        if (++shared.frames_dropped % 100 == 1) {
          std::fprintf(stderr, "basalt-ados: %llu stereo pairs dropped (flow behind)\n",
                       static_cast<unsigned long long>(shared.frames_dropped));
        }
      }
    }
  }
  // End of input: drain the pipeline.
  flow.input_queue.push(nullptr);
  vio.imu_data_queue.push(nullptr);
}

// Optical flow output -> VIO input, recording the tracked-feature count.
void forward_loop(
    tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>& flow_out,
    basalt::VioEstimatorBase& vio, Shared& shared) {
  while (true) {
    basalt::OpticalFlowResult::Ptr r;
    flow_out.pop(r);
    if (r) {
      const size_t n = r->observations.empty() ? 0 : r->observations[0].size();
      std::lock_guard<std::mutex> lock(shared.mutex);
      shared.features_at[r->t_ns] = n;
      while (shared.features_at.size() > 64) {
        shared.features_at.erase(shared.features_at.begin());
      }
      shared.low_feature_run = n < kMinFeatures ? shared.low_feature_run + 1 : 0;
    }
    vio.vision_data_queue.push(r);
    if (!r) return;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s <socket> <calib.json> <config.json>\n",
                 argv[0]);
    return 2;
  }
  claim_stdout();
  const basalt::Calibration<double> calib = load_calib(argv[2]);
  const basalt::VioConfig config = load_config(argv[3]);
  const int fd = connect_unix(argv[1]);

  basalt::OpticalFlowBase::Ptr flow =
      basalt::OpticalFlowFactory::getOpticalFlow(config, calib);
  basalt::VioEstimatorBase::Ptr vio =
      basalt::VioEstimatorFactory::getVioEstimator(
          config, calib, basalt::constants::g, true, false);
  vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  vio->imu_data_queue.set_capacity(kImuQueueCapacity);

  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> flow_out;
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> states;
  flow->output_queue = &flow_out;
  vio->out_state_queue = &states;

  Shared shared;
  std::thread input([&] { input_loop(fd, calib, *flow, *vio, shared); });
  std::thread forward([&] { forward_loop(flow_out, *vio, shared); });

  enum class Status { Initializing, Tracking, Lost };
  Status status = Status::Initializing;
  emit("{\"status\":\"initializing\"}");
  auto set_status = [&](Status s) {
    if (s == status) return;
    status = s;
    emit(s == Status::Tracking ? "{\"status\":\"tracking\"}"
                               : "{\"status\":\"lost\"}");
  };

  int64_t last_state_ms = 0;
  int64_t lost_since_ms = 0;
  int64_t first_frame_ms = 0;
  while (true) {
    basalt::PoseVelBiasState<double>::Ptr st;
    if (!states.try_pop(st)) {
      const int64_t now = now_ms();
      if (first_frame_ms == 0 && shared.frames_in > 0) first_frame_ms = now;
      const bool frames_flowing =
          shared.frames_in > 0 &&
          now - shared.last_frame_ms < kStateTimeout.count();
      if (status == Status::Initializing && first_frame_ms != 0 &&
          now - first_frame_ms > kInitExit.count()) {
        die(3, "no state within the initialisation timeout");
      }
      if (status == Status::Tracking && frames_flowing &&
          now - last_state_ms > kStateTimeout.count()) {
        set_status(Status::Lost);
        lost_since_ms = now;
      }
      if (status == Status::Lost && now - lost_since_ms > kLostExit.count()) {
        die(3, "tracking lost");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    if (!st) break;  // input closed and pipeline drained

    const Sophus::SE3d& T = st->T_w_i;
    if (!T.matrix().allFinite()) {
      set_status(Status::Lost);
      die(3, "estimator diverged (non-finite pose)");
    }
    long features = -1;
    bool low = false;
    {
      std::lock_guard<std::mutex> lock(shared.mutex);
      auto it = shared.features_at.find(st->t_ns);
      if (it != shared.features_at.end()) features = long(it->second);
      low = shared.low_feature_run >= kLowFeatureFrames;
    }
    const int64_t now = now_ms();
    last_state_ms = now;
    if (low) {
      if (status != Status::Lost) lost_since_ms = now;
      set_status(Status::Lost);
      if (now - lost_since_ms > kLostExit.count()) die(3, "tracking lost");
    } else if (features >= long(kMinFeatures)) {
      set_status(Status::Tracking);
    }
    emit(state_line(st->t_ns, T, features));
  }

  input.join();
  forward.join();
  vio->maybe_join();
  ::close(fd);
  return 0;
}
