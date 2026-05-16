// ados_vins_fusion_shim: bridge the plugin's IPC layer to VINS-Fusion.
//
// Mirrors the OpenVINS shim's adapter pattern. The actual estimator
// integration is upstream; this file owns the adapter between the
// plugin's wire format and the VINS-Fusion Estimator class.

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "ipc_channel.hpp"
#include "shm_ring.hpp"

namespace {

void print_usage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --socket <path> --shm <path>"
            << " [--config <yaml>] [--conformance]\n";
}

int run_conformance() {
  std::cerr << "conformance: not yet implemented\n";
  return 1;
}

int run(const std::string& socket_path,
        const std::string& shm_path,
        const std::string& config_path) {
  ados::vision_nav::IpcChannel channel(socket_path);
  ados::vision_nav::ShmRing frames(shm_path);

  // TODO(vendor-build): instantiate VINS-Fusion Estimator with the
  // loaded config, then pump frames + IMU through it. The
  // integration shape mirrors the Python VinsFusionEngine in
  // shim/engine.py.
  (void)channel;
  (void)frames;
  (void)config_path;

  std::cerr << "ados_vins_fusion_shim: vendor binary not yet built. "
               "Source pin: VINS_FUSION_REF cmake variable.\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::string socket_path;
  std::string shm_path;
  std::string config_path;
  bool conformance = false;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--socket" && i + 1 < argc) {
      socket_path = argv[++i];
    } else if (arg == "--shm" && i + 1 < argc) {
      shm_path = argv[++i];
    } else if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--conformance") {
      conformance = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::cerr << "unknown arg: " << arg << "\n";
      print_usage(argv[0]);
      return 64;
    }
  }

  if (conformance) return run_conformance();
  if (socket_path.empty() || shm_path.empty()) {
    print_usage(argv[0]);
    return 64;
  }
  return run(socket_path, shm_path, config_path);
}
