#!/usr/bin/env bash
# Pack a Rust extension (capture or reconstruct) with this machine's payloads,
# for bench testing a release before it is tagged.
#
#   ADOS_SIGNING_KEY=<key> ADOS_SIGNING_KEY_ID=<signer-id> \
#     scripts/pack-local.sh <capture|reconstruct> <out-dir> <payload-base-url>
#
# Builds the extension's binaries for this host's arch (Linux x86_64 or
# aarch64, or Apple Silicon; the reconstruction node with CoreML there) and
# its GCS module, stages them in <out-dir> under their release asset names
# (scripts/<ext>-payloads.tsv), packs the archive with payload rows for this
# arch pointing at <payload-base-url>, and signs it. Serve <out-dir> at that
# URL, then install the signed archive on a node of the same arch.

set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: ADOS_SIGNING_KEY=<key> ADOS_SIGNING_KEY_ID=<signer-id> pack-local.sh <capture|reconstruct> <out-dir> <payload-base-url>" >&2
  exit 2
fi
ext="$1"
case "${ext}" in
  capture | reconstruct) ;;
  *)
    echo "pack-local.sh packs capture or reconstruct, not '${ext}'" >&2
    exit 2
    ;;
esac
for var in ADOS_SIGNING_KEY ADOS_SIGNING_KEY_ID; do
  if [[ -z "${!var:-}" ]]; then
    echo "${var} is required (see scripts/sign.sh)" >&2
    exit 2
  fi
done
if [[ "$3" != https://* ]]; then
  echo "payload base URL must be https:// (the host refuses any other payload source): $3" >&2
  exit 2
fi

case "$(uname -s)-$(uname -m)" in
  Linux-x86_64) arch_os="x86_64-linux" ;;
  Linux-aarch64) arch_os="aarch64-linux" ;;
  Darwin-arm64) arch_os="aarch64-macos" ;;
  *)
    echo "no payload arch for $(uname -s)-$(uname -m)" >&2
    exit 2
    ;;
esac

mkdir -p "$2"
out_dir="$(cd "$2" && pwd)"
base_url="${3%/}"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
ext_dir="${repo_root}/extensions/${ext}"
table="${repo_root}/scripts/${ext}-payloads.tsv"
cd "${repo_root}"

# The binaries this arch ships: the rows of this arch whose asset is
# <crate>-<arch_os>.
crates=()
while read -r asset; do
  crates+=("${asset%-"${arch_os}"}")
done < <(awk -v arch="${arch_os}" '!/^[[:space:]]*(#|$)/ && $3 == arch { print $1 }' "${table}")
if [[ ${#crates[@]} -eq 0 ]]; then
  echo "${ext} ships no binary for ${arch_os}" >&2
  exit 2
fi

for crate in "${crates[@]}"; do
  features=()
  if [[ "${crate}" == "reconstruct-node" && "${arch_os}" == "aarch64-macos" ]]; then
    features=(--features coreml)
  fi
  echo "building ${crate} for ${arch_os}..."
  cargo build --release -p "${crate}" ${features[@]+"${features[@]}"}
  install -m 0755 "target/release/${crate}" "${out_dir}/${crate}-${arch_os}"
done

echo "building the GCS module..."
pnpm "build:${ext}"
cp "${ext_dir}/gcs/assets/re_viewer_bg.wasm" "${out_dir}/re_viewer_bg.wasm"

awk -v arch="${arch_os}" -v base="${base_url}" '
  /^[[:space:]]*(#|$)/ { next }
  $3 == arch || $3 == "-" { printf "%s\t%s\t%s\t%s\t%s/%s\n", $1, $2, $3, $4, base, $1 }
' "${table}" > "${out_dir}/payloads.tsv"

scripts/pack-rust.sh "${ext}" --payload-manifest "${out_dir}/payloads.tsv" --arch-os "${arch_os}"

version="$(grep -E '^version:' "${ext_dir}/manifest.yaml" | head -n1 | sed -E 's/^version:[[:space:]]*"?([^"]*)"?/\1/')"
archive="com.altnautica.${ext}-${version}.adosplug"
mv "${ext_dir}/dist/${archive}" "${out_dir}/${archive}"
scripts/sign.sh "${out_dir}/${archive}"

signed="${out_dir}/${archive%.adosplug}.signed.adosplug"
echo "signer: $(unzip -p "${signed}" SIGNATURE | head -n1)"
if command -v sha256sum >/dev/null; then sha256sum "${signed}"; else shasum -a 256 "${signed}"; fi
echo "serve ${out_dir} at ${base_url}, then install ${signed}"
