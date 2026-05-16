cmake_minimum_required(VERSION 3.16)
project(ados_vins_fusion_shim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Informational upstream pin. The shim is a scaffold (see
# src/main.cpp `run_conformance`) and does not currently link against
# VINS-Fusion; the upstream is a ROS package, not a standalone CMake
# library export. Real integration is future work — the shim ships
# pre-built, signed, and exits with the "not yet implemented"
# sentinel that the conformance gate accepts.
set(VINS_FUSION_REF "be55a937a57436548ddfb1bd324bc1e9a9e828e0" CACHE STRING "VINS-Fusion upstream ref (informational)")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

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
)

install(TARGETS ados_vins_fusion_shim DESTINATION bin)
