cmake_minimum_required(VERSION 3.16)
project(ados_vins_fusion_shim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pin the upstream commit. The CI workflow passes this in on the
# command line so a release tag locks the exact source revision.
# VINS-Fusion does not publish formal release tags; the canonical
# pin is a commit sha on the master branch. Update the default below
# only via a release-bump PR with a one-line rationale.
set(VINS_FUSION_REF "master" CACHE STRING "VINS-Fusion upstream ref")

include(FetchContent)
FetchContent_Declare(
  vins_fusion
  GIT_REPOSITORY https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
  GIT_TAG ${VINS_FUSION_REF}
)

# Same dependency set as OpenVINS plus Ceres for the bundle
# adjustment back-end.
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)
find_package(msgpack REQUIRED)

FetchContent_MakeAvailable(vins_fusion)

add_executable(ados_vins_fusion_shim
  src/main.cpp
  src/ipc_channel.cpp
  src/shm_ring.cpp
)

target_include_directories(ados_vins_fusion_shim PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

target_link_libraries(ados_vins_fusion_shim PRIVATE
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Boost::system
  Boost::thread
  Boost::filesystem
  ${CERES_LIBRARIES}
  Threads::Threads
  msgpackc-cxx
)

install(TARGETS ados_vins_fusion_shim DESTINATION bin)
