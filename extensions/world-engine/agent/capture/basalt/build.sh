#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Build basalt-ados for one Linux architecture. Native builds only: run on an
# x86_64 host for --arch x86_64 and on an aarch64 host (or an aarch64
# container, see README.md) for --arch aarch64.
#
#   ./build.sh --arch x86_64|aarch64 [--jobs N] [--march M] [--out DIR]
#
# Dependencies are built by upstream Basalt's pinned vcpkg into the build
# directory; nothing is installed system-wide. Host tools needed: a C++17
# compiler, cmake >= 3.24, git, curl, zip, unzip, tar, pkg-config (ninja is
# used when present). The binary lands in <out>/bin/<arch>-linux/basalt-ados.

set -euo pipefail

usage() {
  echo "usage: ./build.sh --arch x86_64|aarch64 [--jobs N] [--march M] [--out DIR]" >&2
  exit 2
}

ARCH=""
JOBS=2
MARCH=""
OUT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --arch) ARCH="${2:-}"; shift 2 ;;
    --jobs) JOBS="${2:-}"; shift 2 ;;
    --march) MARCH="${2:-}"; shift 2 ;;
    --out) OUT="${2:-}"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "unknown argument: $1" >&2; usage ;;
  esac
done

case "$ARCH" in
  x86_64) TRIPLET=x64-linux-release; DEFAULT_MARCH=x86-64-v2 ;;
  aarch64) TRIPLET=arm64-linux-release; DEFAULT_MARCH=armv8-a ;;
  *) echo "--arch must be x86_64 or aarch64" >&2; usage ;;
esac
MARCH="${MARCH:-$DEFAULT_MARCH}"

HOST_ARCH="$(uname -m)"
[ "$HOST_ARCH" = arm64 ] && HOST_ARCH=aarch64
if [ "$(uname -s)" != Linux ] || [ "$HOST_ARCH" != "$ARCH" ]; then
  echo "native build only: host is $(uname -s)/$HOST_ARCH, asked for Linux/$ARCH" >&2
  exit 2
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
UPSTREAM="$HERE/upstream"
BUILD="${BUILD_DIR:-$HERE/build/$ARCH}"
OUT="${OUT:-$HERE/build/out}"

if [ ! -f "$UPSTREAM/CMakeLists.txt" ]; then
  echo "Basalt sources missing; from the repository root run:" >&2
  echo "  git submodule update --init extensions/world-engine/agent/capture/basalt/upstream" >&2
  exit 2
fi

for tool in cmake git curl zip unzip tar pkg-config; do
  command -v "$tool" >/dev/null || { echo "missing host tool: $tool" >&2; exit 2; }
done

VCPKG_ROOT="$UPSTREAM/thirdparty/vcpkg"
if [ ! -f "$VCPKG_ROOT/bootstrap-vcpkg.sh" ]; then
  # Full history: vcpkg resolves the manifest baseline from it.
  git -C "$UPSTREAM" submodule update --init thirdparty/vcpkg
fi
if [ "$(git -C "$VCPKG_ROOT" rev-parse --is-shallow-repository)" = true ]; then
  git -C "$VCPKG_ROOT" fetch --unshallow
fi
if [ ! -x "$VCPKG_ROOT/vcpkg" ]; then
  "$VCPKG_ROOT/bootstrap-vcpkg.sh" -disableMetrics
fi

# Keep every vcpkg byte under the build directory, one port at a time.
mkdir -p "$BUILD" "$HERE/build/vcpkg-downloads" "$HERE/build/vcpkg-archives"
export VCPKG_ROOT
export VCPKG_DISABLE_METRICS=1
export VCPKG_MAX_CONCURRENCY="$JOBS"
export VCPKG_DOWNLOADS="${VCPKG_DOWNLOADS:-$HERE/build/vcpkg-downloads}"
export VCPKG_DEFAULT_BINARY_CACHE="${VCPKG_DEFAULT_BINARY_CACHE:-$HERE/build/vcpkg-archives}"
VCPKG_OPTS="--clean-after-build;--x-buildtrees-root=$BUILD/vcpkg-buildtrees;--x-packages-root=$BUILD/vcpkg-packages"

GENERATOR=()
command -v ninja >/dev/null && GENERATOR=(-G Ninja)

cmake -S "$HERE" -B "$BUILD" "${GENERATOR[@]}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
  -DVCPKG_TARGET_TRIPLET="$TRIPLET" \
  -DVCPKG_INSTALLED_DIR="$BUILD/vcpkg_installed" \
  -DVCPKG_INSTALL_OPTIONS="$VCPKG_OPTS" \
  -DADOS_CXX_MARCH="$MARCH"
cmake --build "$BUILD" --target basalt-ados -j "$JOBS"

DEST="$OUT/bin/$ARCH-linux"
mkdir -p "$DEST"
install -m 0755 "$BUILD/basalt-ados" "$DEST/basalt-ados"
strip "$DEST/basalt-ados"
echo "built $DEST/basalt-ados"
