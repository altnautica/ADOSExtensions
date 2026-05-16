cmake_minimum_required(VERSION 3.16)
project(ados_openvins_shim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pin the upstream source the shim targets. The shim itself is a
# scaffold (see src/main.cpp `run_conformance`) and does not currently
# link against the upstream library; OpenVINS upstream is structured
# as a catkin/colcon ROS package, not a standalone CMake export, so
# FetchContent + open_vins::* targets do not resolve. Real integration
# is future work — the shim ships pre-built, signed, and exits with
# the "not yet implemented" sentinel that the conformance gate accepts.
set(OPENVINS_VERSION "v2.7" CACHE STRING "OpenVINS upstream tag (informational)")

# Toolchain deps minimal for the scaffold. The current sources use
# only POSIX + std and exit with the "not yet implemented" sentinel.
# Eigen / OpenCV / Boost / Ceres become required when the upstream
# estimator integration lands; until then keeping them out of the
# build matches what the code actually consumes and keeps the arm64
# sysroot install lean.
find_package(Threads REQUIRED)

add_executable(ados_openvins_shim
  src/main.cpp
  src/ipc_channel.cpp
  src/shm_ring.cpp
)

target_link_libraries(ados_openvins_shim PRIVATE
  Threads::Threads
)

install(TARGETS ados_openvins_shim DESTINATION bin)
