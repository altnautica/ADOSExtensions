#!/usr/bin/env bash
#
# Build, pack and sign a World Engine archive on an Apple Silicon Mac for bench
# testing, the same way the release workflow does for one arch.
#
# Usage:
#   ADOS_SIGNING_KEY=/path/to/ed25519.pem \
#     scripts/pack-world-engine-local.sh <out-dir> <payload-base-url>
#
# Example:
#   ADOS_SIGNING_KEY=~/keys/bench.pem \
#     scripts/pack-world-engine-local.sh /tmp/we-bench https://bench.example.com/we
#
# What it does:
#   1. Builds the aarch64-macos binaries this Mac can produce natively:
#      world-engine-link, and world-engine-node with `--features coreml`
#      (the same features as the release workflow's macos-14 leg).
#   2. Builds the GCS module (`pnpm build:world-engine`).
#   3. Copies every payload into <out-dir> under its release asset name and
#      writes <out-dir>/payloads.tsv from scripts/world-engine-payloads.tsv,
#      keeping the aarch64-macos and arch-independent rows, each pointing at
#      <payload-base-url>/<asset-name>.
#   4. Packs with `scripts/pack-rust.sh --payload-manifest ... --arch-os
#      aarch64-macos` and signs with scripts/sign.sh. The unsigned and signed
#      archives land in <out-dir>.
#
# Serve <out-dir> at <payload-base-url> and install the signed archive on a Mac
# node. The archive covers aarch64-macos only: a Linux node refuses it with
# `incompatible: no binary for <arch-os>`.
#
# The payload URL must satisfy the host, which refuses anything else at
# install: an https:// URL on its download allowlist (github.com and the GitHub
# asset hosts, *.amazonaws.com, the convex hosts, or `localhost`), served with
# a certificate that chains to a public root (the host trusts the bundled
# webpki roots only). pack-rust.sh rejects a non-https URL up front.
#
# Signing: ADOS_SIGNING_KEY is passed straight through to scripts/sign.sh (a
# key file path, or base64 key material with ADOS_SIGNING_KEY_INLINE=1); this
# script never reads the key itself. ADOS_SIGNING_KEY_ID sets the signer id
# (sign.sh defaults to altnautica-2026-A). A node verifies the signature only
# against the public key it holds for that id, so an archive signed with a
# bench key installs on a node that trusts that key or runs with
# ADOS_PLUGIN_REQUIRE_SIGNED=0.

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: ADOS_SIGNING_KEY=<key> pack-world-engine-local.sh <out-dir> <payload-base-url>" >&2
  exit 2
fi
if [[ "$(uname -s)-$(uname -m)" != "Darwin-arm64" ]]; then
  echo "this builds the aarch64-macos payloads natively; run it on an Apple Silicon Mac" >&2
  exit 2
fi
# Checked before the long build, not discovered after it.
if [[ -z "${ADOS_SIGNING_KEY:-}" ]]; then
  echo "ADOS_SIGNING_KEY is required (see scripts/sign.sh)" >&2
  exit 2
fi

if [[ "$2" != https://* ]]; then
  echo "payload base URL must be https:// (the host refuses any other payload source): $2" >&2
  exit 2
fi
mkdir -p "$1"
out_dir="$(cd "$1" && pwd)"
base_url="${2%/}"
arch_os="aarch64-macos"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
ext_dir="${repo_root}/extensions/world-engine"

cd "${repo_root}"

echo "building world-engine-link and world-engine-node (coreml) for ${arch_os}..."
cargo build --release -p world-engine-link
cargo build --release -p world-engine-node --features coreml

echo "building the GCS module..."
pnpm build:world-engine

for crate in world-engine-link world-engine-node; do
  install -m 0755 "target/release/${crate}" "${out_dir}/${crate}-${arch_os}"
done
cp "${ext_dir}/gcs/assets/re_viewer_bg.wasm" "${out_dir}/re_viewer_bg.wasm"

awk -v arch="${arch_os}" -v base="${base_url}" '
  /^[[:space:]]*(#|$)/ { next }
  $3 == arch || $3 == "-" { printf "%s\t%s\t%s\t%s\t%s/%s\n", $1, $2, $3, $4, base, $1 }
' scripts/world-engine-payloads.tsv > "${out_dir}/payloads.tsv"

scripts/pack-rust.sh world-engine --payload-manifest "${out_dir}/payloads.tsv" --arch-os "${arch_os}"

version="$(grep -E '^version:' "${ext_dir}/manifest.yaml" | head -n1 | sed -E 's/^version:[[:space:]]*"?([^"]*)"?/\1/')"
archive="com.altnautica.world-engine-${version}.adosplug"
mv "${ext_dir}/dist/${archive}" "${out_dir}/${archive}"
scripts/sign.sh "${out_dir}/${archive}"

signed="${out_dir}/${archive%.adosplug}.signed.adosplug"
echo "signer: $(unzip -p "${signed}" SIGNATURE | head -n1)"
shasum -a 256 "${signed}"
echo "serve ${out_dir} at ${base_url}, then install ${signed}"
