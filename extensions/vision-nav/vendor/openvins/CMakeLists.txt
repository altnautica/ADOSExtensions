cmake_minimum_required(VERSION 3.16)
project(ados_openvins_shim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pin the upstream commit. The CI workflow passes this in on the
# command line so a release tag locks the exact source revision.
set(OPENVINS_VERSION "v2.7" CACHE STRING "OpenVINS upstream tag or sha")

include(FetchContent)
FetchContent_Declare(
  open_vins
  GIT_REPOSITORY https://github.com/rpng/open_vins.git
  GIT_TAG ${OPENVINS_VERSION}
)

# OpenVINS depends on Eigen, OpenCV, and Boost. The CI sysroot
# pre-provisions these so FetchContent only needs to pull the
# estimator source itself.
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(Threads REQUIRED)

FetchContent_MakeAvailable(open_vins)

# msgpack-c for the UDS channel.
find_package(msgpack REQUIRED)

# Single-binary build. The main translation unit hooks the shim IPC
# layer to the upstream VioManager.
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
  open_vins::ov_msckf
  open_vins::ov_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Boost::system
  Boost::thread
  Boost::filesystem
  Threads::Threads
  msgpackc-cxx
)

install(TARGETS ados_openvins_shim DESTINATION bin)
