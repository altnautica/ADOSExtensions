#!/usr/bin/env bash
# Build the librealsense2 shared library (Apache-2.0, upstream
# realsenseai/librealsense) at a pinned tag into a payload directory, for the
# lidar-slam depth source to load at run time (`navigation.depth.source =
# "realsense"`: Intel RealSense D435i / D455).
#
# Usage: build-librealsense2.sh --arch x86_64|aarch64 [--out DIR] [--src DIR] [--jobs N]
#
# Output, under DIR (default ./payload):
#   lib/<arch>-linux/librealsense2.so            stripped shared library
#   lib/<arch>-linux/LICENSE-librealsense2.txt   upstream licence and third-party notices
# and one line per file with its sha256 and size, for the manifest payloads.
#
# The library talks to the camera through libusb in user space (the RSUSB
# backend), so it needs no kernel patches on any board; it links the system
# libusb-1.0 dynamically (a board needs libusb-1.0-0, present on stock
# Debian and Ubuntu images) and a udev rule giving the service user access to
# the camera. Update checks, usage statistics, examples, tools and the
# recorder's ROS2 bag backend are off: the library opens no network
# connection.
#
# Building aarch64 on an x86_64 host cross-compiles with
# aarch64-linux-gnu-gcc/g++ (Debian/Ubuntu: g++-aarch64-linux-gnu); CC/CXX in
# the environment override the compilers.
#
# Release: each leg of .github/workflows/navigation-release.yml runs
# `build-librealsense2.sh --arch <leg arch>` inside the manylinux_2_28 image
# of that arch, so the library needs nothing newer than glibc 2.28, and
# stages the two files as the assets named in scripts/navigation-payloads.tsv.
# Inside that image (run as root) the script installs the libusb headers
# with dnf when they are missing.
set -euo pipefail

TAG="v2.58.4"
COMMIT="34d6c778e1134d8505adcd56bb57acbe7598a459"
REPO="https://github.com/realsenseai/librealsense.git"
LIB="librealsense2.so"

ARCH=""
OUT="$(pwd)/payload"
SRC=""
JOBS=2

while [ $# -gt 0 ]; do
  case "$1" in
    --arch) shift; ARCH="${1:?--arch needs x86_64 or aarch64}" ;;
    --out) shift; OUT="${1:?--out needs a directory}" ;;
    --src) shift; SRC="${1:?--src needs a directory}" ;;
    --jobs|-j) shift; JOBS="${1:?--jobs needs a number}" ;;
    -h|--help) sed -n '2,31p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

case "$ARCH" in
  x86_64|aarch64) ;;
  *) echo "--arch must be x86_64 or aarch64" >&2; exit 2 ;;
esac
[ "$(uname -s)" = "Linux" ] || { echo "librealsense2 builds on Linux only" >&2; exit 2; }

HOST_ARCH="$(uname -m)"
[ "$HOST_ARCH" = "arm64" ] && HOST_ARCH=aarch64
CMAKE_CROSS=()
STRIP=strip
if [ "$ARCH" != "$HOST_ARCH" ]; then
  [ "$ARCH" = "aarch64" ] || { echo "cross builds target aarch64 only" >&2; exit 2; }
  CC="${CC:-aarch64-linux-gnu-gcc}"
  CXX="${CXX:-aarch64-linux-gnu-g++}"
  STRIP="${STRIP_BIN:-aarch64-linux-gnu-strip}"
  for tool in "$CC" "$CXX" "$STRIP"; do
    command -v "$tool" >/dev/null || { echo "missing cross tool $tool" >&2; exit 2; }
  done
  CMAKE_CROSS=(-DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64
    -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX")
fi

# libusb-1.0 headers for the RSUSB backend. Without them librealsense would
# build its bundled copy (LGPL-2.1) into the library; linking the system one
# keeps it a separate, replaceable shared object.
if [ ! -e /usr/include/libusb-1.0/libusb.h ] && [ "$ARCH" = "$HOST_ARCH" ]; then
  if [ "$(id -u)" = 0 ] && command -v dnf >/dev/null; then
    dnf install -y -q libusbx-devel >/dev/null
  else
    echo "libusb-1.0 headers missing (/usr/include/libusb-1.0/libusb.h): install libusb-1.0-0-dev or libusbx-devel" >&2
    exit 2
  fi
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
if [ -z "$SRC" ]; then
  SRC="$WORK/librealsense"
  git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$TAG" "$REPO" "$SRC"
fi
HEAD="$(git -C "$SRC" rev-parse HEAD)"
if [ "$HEAD" != "$COMMIT" ]; then
  echo "librealsense checkout is $HEAD, expected $COMMIT ($TAG)" >&2
  exit 1
fi

cmake -S "$SRC" -B "$WORK/build" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON -DFORCE_RSUSB_BACKEND=ON -DUSE_EXTERNAL_USB=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_TOOLS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WITH_DDS=OFF -DBUILD_ROSBAG2=OFF -DCHECK_FOR_UPDATES=OFF \
  -DENABLE_CCACHE=OFF -DBUILD_WITH_CUDA=OFF \
  ${CMAKE_CROSS[@]+"${CMAKE_CROSS[@]}"} >/dev/null
cmake --build "$WORK/build" --target realsense2 -j "$JOBS"

BUILT="$(find "$WORK/build" -name "$LIB.*" -type f | head -n 1)"
[ -n "$BUILT" ] || { echo "$LIB was not produced" >&2; exit 1; }

DEST="$OUT/lib/$ARCH-linux"
mkdir -p "$DEST"
cp "$BUILT" "$DEST/$LIB"
"$STRIP" --strip-unneeded "$DEST/$LIB"
readelf -d "$DEST/$LIB" | grep -q 'NEEDED.*libusb-1\.0\.so' || {
  echo "$LIB does not link the system libusb-1.0 (bundled copy built in?)" >&2
  exit 1
}
{
  cat "$SRC/LICENSE"
  printf '\n\n'
  cat "$SRC/NOTICE.md"
} > "$DEST/LICENSE-librealsense2.txt"

for f in "$DEST/$LIB" "$DEST/LICENSE-librealsense2.txt"; do
  printf '%s  sha256=%s  size_bytes=%s\n' "${f#"$OUT"/}" \
    "$(sha256sum "$f" | cut -d' ' -f1)" "$(stat -c %s "$f")"
done
