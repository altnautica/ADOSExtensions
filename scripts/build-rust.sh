#!/usr/bin/env bash
#
# Cross-compile a Rust extension agent half to the drone-board target.
#
# Usage:
#   scripts/build-rust.sh <crate-name> [target]
#
# Examples:
#   scripts/build-rust.sh hello-rust
#   scripts/build-rust.sh hello-rust aarch64-unknown-linux-musl
#
# Drone-class ADOS boards are aarch64 Linux. The default target is the static
# musl triple so the binary carries no libc dependency and runs on any of the
# supported boards without matching a glibc version. The build uses the release
# profile (stripped, LTO) so the binary stays small in the .adosplug archive.
#
# A static aarch64 build needs an aarch64 musl linker. Apple's `ld` is not one:
# it rejects the GNU-style arguments rustc passes for this target, so a plain
# macOS build compiles the whole dependency graph and then fails at the link
# step. Set the linker with the usual cargo env var, for example:
#
#   CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=aarch64-linux-musl-gcc \
#     scripts/build-rust.sh hello-rust
#
# `zig cc` also works as a cross-linker where no musl toolchain is installed.
# It ships its own musl start files, so the rust-provided ones have to be
# turned off or the link fails on duplicate `_start` symbols, and it does not
# implement the `--fix-cortex-a53-843419` erratum flag rustc passes, so a thin
# wrapper has to drop that argument:
#
#   CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=/path/to/zig-cc-wrapper \
#   CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_RUSTFLAGS="-C link-self-contained=no" \
#     scripts/build-rust.sh hello-rust
#
# or build inside a container/CI runner that has the aarch64 musl cross tool
# chain installed (the release workflow uses ubuntu-24.04-arm with musl-tools).
# The script does not install a toolchain; it builds with whatever the
# environment provides and reports the binary path on success.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: build-rust.sh <crate-name> [target]" >&2
  exit 2
fi

crate="$1"
target="${2:-aarch64-unknown-linux-musl}"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"

echo "building crate '${crate}' for target '${target}' (release)..."

# Ensure the target's std is present. `rustup target add` is idempotent; it is
# a no-op when the target is already installed.
if command -v rustup >/dev/null 2>&1; then
  rustup target add "${target}" >/dev/null 2>&1 || true
fi

(cd "${repo_root}" && cargo build --release --target "${target}" -p "${crate}")

bin_path="${repo_root}/target/${target}/release/${crate}"
if [[ ! -f "${bin_path}" ]]; then
  echo "build reported success but binary not found at ${bin_path}" >&2
  exit 1
fi

echo "built ${bin_path}"
file "${bin_path}" || true
