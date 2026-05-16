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

# Toolchain deps for the scaffold (OpenCV / Eigen / Boost / msgpack
# are present so future revisions of the shim can adopt the real
# estimator without another CI sweep).
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(Threads REQUIRED)

add_executable(ados_openvins_shim
  src/main.cpp
  src/ipc_channel.cpp
  src/shm_ring.cpp
)

target_include_directories(ados_openvins_shim PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(ados_openvins_shim PRIVATE
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Boost::system
  Boost::thread
  Boost::filesystem
  Threads::Threads
)

install(TARGETS ados_openvins_shim DESTINATION bin)
