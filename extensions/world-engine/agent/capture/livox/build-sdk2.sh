#!/usr/bin/env bash
# Build the Livox SDK2 shared library (MIT, upstream Livox-SDK/Livox-SDK2) at a
# pinned tag into a payload directory, for the ados-livox `sdk` feature to load
# at run time.
#
# Usage: build-sdk2.sh --arch x86_64|aarch64 [--out DIR] [--src DIR] [--jobs N]
#
# Output, under DIR (default ./payload):
#   lib/<arch>-linux/liblivox_lidar_sdk_shared.so   stripped shared library
#   lib/<arch>-linux/LICENSE-livox-sdk2.txt        upstream licence text
# and one line per file with its sha256 and size, for the manifest payloads.
#
# Building aarch64 on an x86_64 host cross-compiles with
# aarch64-linux-gnu-gcc/g++ (Debian/Ubuntu: g++-aarch64-linux-gnu); CC/CXX in
# the environment override the compilers.
set -euo pipefail

TAG="v1.2.5"
COMMIT="6a940156dd7151c3ab6a52442d86bc83613bd11b"
REPO="https://github.com/Livox-SDK/Livox-SDK2.git"
LIB="liblivox_lidar_sdk_shared.so"

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
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

case "$ARCH" in
  x86_64|aarch64) ;;
  *) echo "--arch must be x86_64 or aarch64" >&2; exit 2 ;;
esac
[ "$(uname -s)" = "Linux" ] || { echo "Livox SDK2 builds on Linux only" >&2; exit 2; }

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

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
if [ -z "$SRC" ]; then
  SRC="$WORK/Livox-SDK2"
  git clone --quiet --depth 1 --branch "$TAG" "$REPO" "$SRC"
fi
HEAD="$(git -C "$SRC" rev-parse HEAD)"
if [ "$HEAD" != "$COMMIT" ]; then
  echo "Livox-SDK2 checkout is $HEAD, expected $COMMIT ($TAG)" >&2
  exit 1
fi

# The v1.2.5 sources use fixed-width integer types without including
# <cstdint>, which newer GCC no longer pulls in transitively.
cmake -S "$SRC" -B "$WORK/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-include cstdint" ${CMAKE_CROSS[@]+"${CMAKE_CROSS[@]}"} >/dev/null
cmake --build "$WORK/build" --target livox_lidar_sdk_shared -j "$JOBS"

BUILT="$(find "$WORK/build" -name "$LIB" -type f | head -n 1)"
[ -n "$BUILT" ] || { echo "$LIB was not produced" >&2; exit 1; }

DEST="$OUT/lib/$ARCH-linux"
mkdir -p "$DEST"
cp "$BUILT" "$DEST/$LIB"
"$STRIP" --strip-unneeded "$DEST/$LIB"
cp "$SRC/LICENSE.txt" "$DEST/LICENSE-livox-sdk2.txt"

for f in "$DEST/$LIB" "$DEST/LICENSE-livox-sdk2.txt"; do
  printf '%s  sha256=%s  size_bytes=%s\n' "${f#"$OUT"/}" \
    "$(sha256sum "$f" | cut -d' ' -f1)" "$(stat -c %s "$f")"
done
