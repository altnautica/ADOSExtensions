# SPDX-License-Identifier: BSD-3-Clause
#
# basalt-ados: Basalt stereo-inertial VIO behind a Unix-socket input.
#
# Compiles only the upstream Basalt sources the VIO pipeline needs (optical
# flow, linearisation, square-root keypoint VIO) into a static library, with
# float instantiations only, and links the `basalt-ados` executable against
# it. No GUI, Pangolin, rosbag, calibration or mapping code is built. The
# dependencies come from the vcpkg manifest next to this file (see build.sh).

cmake_minimum_required(VERSION 3.24)

project(basalt_ados LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ADOS_CXX_MARCH "" CACHE STRING "Value for -march (empty: compiler default)")
if(ADOS_CXX_MARCH)
  add_compile_options("-march=${ADOS_CXX_MARCH}")
endif()

set(BASALT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/upstream")
if(NOT EXISTS "${BASALT_DIR}/include/basalt/vi_estimator/vio_estimator.h")
  message(FATAL_ERROR "Basalt sources missing: git submodule update --init "
                      "extensions/world-engine/agent/capture/basalt/upstream")
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc features2d)
find_package(fmt CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(opengv CONFIG REQUIRED)
find_package(basalt-headers CONFIG REQUIRED)

add_library(basalt_vio STATIC
  ${BASALT_DIR}/src/linearization/landmark_block.cpp
  ${BASALT_DIR}/src/linearization/linearization_abs_qr.cpp
  ${BASALT_DIR}/src/linearization/linearization_abs_sc.cpp
  ${BASALT_DIR}/src/linearization/linearization_base.cpp
  ${BASALT_DIR}/src/linearization/linearization_rel_sc.cpp
  ${BASALT_DIR}/src/optical_flow/optical_flow.cpp
  ${BASALT_DIR}/src/utils/keypoints.cpp
  ${BASALT_DIR}/src/utils/system_utils.cpp
  ${BASALT_DIR}/src/utils/time_utils.cpp
  ${BASALT_DIR}/src/utils/vio_config.cpp
  ${BASALT_DIR}/src/vi_estimator/ba_base.cpp
  ${BASALT_DIR}/src/vi_estimator/landmark_database.cpp
  ${BASALT_DIR}/src/vi_estimator/marg_helper.cpp
  ${BASALT_DIR}/src/vi_estimator/sc_ba_base.cpp
  ${BASALT_DIR}/src/vi_estimator/sqrt_ba_base.cpp
  ${BASALT_DIR}/src/vi_estimator/sqrt_keypoint_vio.cpp
  ${BASALT_DIR}/src/vi_estimator/sqrt_keypoint_vo.cpp
  ${BASALT_DIR}/src/vi_estimator/vio_estimator.cpp
)
target_include_directories(basalt_vio PUBLIC "${BASALT_DIR}/include")
# Float only: the estimator runs in float (as upstream's default), and
# skipping the double instantiations halves compile time and memory.
target_compile_definitions(basalt_vio PUBLIC
  BASALT_INSTANTIATIONS_FLOAT
  EIGEN_DONT_PARALLELIZE)
target_compile_options(basalt_vio PRIVATE -Wno-deprecated-declarations)
target_link_libraries(basalt_vio
  PUBLIC
    Eigen3::Eigen
    basalt::basalt-headers
    TBB::tbb
    opencv_core opencv_imgproc opencv_features2d
    fmt::fmt
    Threads::Threads
  PRIVATE
    magic_enum::magic_enum
    nlohmann_json::nlohmann_json
    opengv)

add_executable(basalt-ados ados_input.cpp)
target_compile_options(basalt-ados PRIVATE -Wall -Wextra)
target_link_libraries(basalt-ados PRIVATE basalt_vio)
# glibc stays dynamic; the C++ runtime and every library above are static,
# so the binary runs on any glibc at least as new as the build host's.
target_link_options(basalt-ados PRIVATE -static-libstdc++ -static-libgcc)

install(TARGETS basalt-ados RUNTIME DESTINATION bin)
