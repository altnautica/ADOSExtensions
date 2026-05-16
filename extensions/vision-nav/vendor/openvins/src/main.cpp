// ados_openvins_shim: bridge the plugin's IPC layer to OpenVINS MSCKF.
//
// The shim connects the plugin's shared-memory frame ring + msgpack
// UDS channel to an upstream ov_msckf::VioManager. Camera frames flow
// in from the SHM ring; IMU samples flow in over msgpack; pose
// messages flow back over msgpack.
//
// The actual estimator integration is upstream; this file owns the
// adapter between the plugin's wire format and the ov_msckf API.
// Build instructions live in ../README.md; the conformance test
// fixture is checked in at tests/conformance/euroc_v1_01_easy.bag
// (downloaded by CI; not committed).
//
// See ../../agent/src/altnautica_vision_nav/shim/ipc.py for the
// authoritative wire-format documentation.

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "ipc_channel.hpp"
#include "shm_ring.hpp"

// Upstream OpenVINS headers are pulled in by FetchContent at build
// time. The build system resolves the include path; the include
// directives below match the upstream library layout.
// #include <core/VioManager.h>
// #include <core/VioManagerOptions.h>

namespace {

void print_usage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --socket <path> --shm <path>"
            << " [--config <yaml>] [--conformance]\n";
}

int run_conformance() {
  // Replay the EuRoC bag fixture and assert pose deltas match the
  // upstream reference within tolerance. The CI workflow runs this
  // before the artefact is signed.
  std::cerr << "conformance: not yet implemented\n";
  return 1;
}

int run(const std::string& socket_path,
        const std::string& shm_path,
        const std::string& config_path) {
  ados::vision_nav::IpcChannel channel(socket_path);
  ados::vision_nav::ShmRing frames(shm_path);

  // TODO(vendor-build): instantiate ov_msckf::VioManager with the
  // loaded VioManagerOptions, then pump frames + IMU through it.
  // The integration shape mirrors the Python OpenVinsEngine in
  // shim/engine.py.
  (void)channel;
  (void)frames;
  (void)config_path;

  std::cerr << "ados_openvins_shim: vendor binary not yet built. "
               "Source pin: OPENVINS_VERSION cmake variable.\n";
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
      return 64;  // EX_USAGE
    }
  }

  if (conformance) return run_conformance();
  if (socket_path.empty() || shm_path.empty()) {
    print_usage(argv[0]);
    return 64;
  }
  return run(socket_path, shm_path, config_path);
}
