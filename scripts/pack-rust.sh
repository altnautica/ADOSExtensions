#!/usr/bin/env bash
#
# Pack a Rust extension into an unsigned .adosplug archive.
#
# Usage:
#   scripts/pack-rust.sh <extension-folder-or-path> [target]
#
# Examples:
#   scripts/pack-rust.sh _hello-rust
#   scripts/pack-rust.sh _hello-rust aarch64-unknown-linux-musl
#   scripts/pack-rust.sh ./extensions/_hello-rust
#
# The script:
#   1. Reads the manifest's agent.entrypoint, which for a rust runtime is the
#      path the binary takes inside the archive (e.g. bin/hello-rust).
#   2. Cross-compiles the crate for the target via scripts/build-rust.sh.
#   3. Stages the manifest, the compiled binary at agent.entrypoint, and any
#      assets (locales, config-schema.json, README, gcs bundle if present).
#   4. Zips the staging tree into dist/<plugin-id>-<version>.adosplug.
#
# Signing is the separate, final step and is unchanged across runtimes:
#   ADOS_SIGNING_KEY=/path/to/key.ed25519 scripts/sign.sh dist/<plugin>.adosplug
#
# The signature covers every archive entry, so the binary at agent.entrypoint
# is protected by the same canonical-payload-hash signature as the manifest.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: pack-rust.sh <extension-folder-or-path> [target]" >&2
  exit 2
fi

arg="$1"
target="${2:-aarch64-unknown-linux-musl}"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"

# Shared exclude list + the entrypoint-existence guard (see _pack-exclude.sh).
source "$(dirname "$0")/_pack-exclude.sh"

# Resolve the extension directory the same way pack.sh does.
if [[ -d "${arg}" && -f "${arg}/manifest.yaml" ]]; then
  ext_dir="$(cd "${arg}" && pwd)"
elif [[ -d "${repo_root}/extensions/${arg}" ]]; then
  ext_dir="${repo_root}/extensions/${arg}"
else
  echo "extension not found: ${arg}" >&2
  echo "  tried: ${arg} and ${repo_root}/extensions/${arg}" >&2
  exit 2
fi

manifest_src="${ext_dir}/manifest.yaml"
if [[ ! -f "${manifest_src}" ]]; then
  echo "manifest not found: ${manifest_src}" >&2
  exit 2
fi

# Parse id, version, runtime, and the entrypoint (the in-archive binary path).
plugin_id="$(grep -E '^id:' "${manifest_src}" | head -n1 | sed -E 's/^id:[[:space:]]*//; s/[[:space:]]*$//')"
plugin_version="$(grep -E '^version:' "${manifest_src}" | head -n1 | sed -E 's/^version:[[:space:]]*"?([^"]*)"?/\1/')"
runtime="$(grep -E '^[[:space:]]*runtime:' "${manifest_src}" | head -n1 | sed -E 's/^[[:space:]]*runtime:[[:space:]]*"?([^"]*)"?/\1/')"
entrypoint="$(grep -E '^[[:space:]]*entrypoint:' "${manifest_src}" | head -n1 | sed -E 's/^[[:space:]]*entrypoint:[[:space:]]*"?([^"]*)"?/\1/')"

if [[ -z "${plugin_id}" || -z "${plugin_version}" ]]; then
  echo "could not parse plugin id/version from manifest" >&2
  exit 1
fi
if [[ "${runtime}" != "rust" ]]; then
  echo "manifest agent.runtime is '${runtime:-python}', not 'rust'; use scripts/pack.sh" >&2
  exit 1
fi
if [[ -z "${entrypoint}" ]]; then
  echo "manifest has no agent.entrypoint" >&2
  exit 1
fi

# The crate name is the directory under the extension's agent/ folder. The
# convention is one crate per extension agent half, named after the binary the
# entrypoint references.
crate="$(basename "${entrypoint}")"

# Build the binary for the requested target.
"${repo_root}/scripts/build-rust.sh" "${crate}" "${target}"
bin_path="${repo_root}/target/${target}/release/${crate}"
if [[ ! -f "${bin_path}" ]]; then
  echo "expected binary not found: ${bin_path}" >&2
  exit 1
fi

# Build the GCS bundle if this extension has a GCS half (esbuild ->
# plugin.bundle.js). This packer used to rely on a stale, un-tracked local
# bundle; it now builds the same way pack.sh does, so a clean CI checkout
# ships a real bundle instead of a manifest pointing at a missing entrypoint.
if [[ -d "${ext_dir}/gcs" ]]; then
  echo "building gcs bundle..."
  (cd "${ext_dir}/gcs" && pnpm build)
fi

archive_name="${plugin_id}-${plugin_version}.adosplug"
dist_dir="${ext_dir}/dist"
mkdir -p "${dist_dir}"
archive_path="${dist_dir}/${archive_name}"
rm -f "${archive_path}"

stage="$(mktemp -d)"
trap 'rm -rf "${stage}"' EXIT

# Stage the manifest and assets. The shared exclude list plus 'agent' (the Rust
# source tree; the compiled binary is staged separately) and 'target' (the
# build output).
rsync -a "${PACK_RSYNC_EXCLUDES[@]}" \
  --exclude 'target' \
  --exclude 'agent' \
  "${ext_dir}/" "${stage}/"

# Place the compiled binary at the manifest entrypoint path inside the archive.
mkdir -p "${stage}/$(dirname "${entrypoint}")"
cp "${bin_path}" "${stage}/${entrypoint}"
chmod 0755 "${stage}/${entrypoint}"

# Never publish a half-archive: the agent binary and, when declared, the built
# GCS bundle must both be present in the stage.
assert_entrypoint_in_stage "${stage}" "${entrypoint}" "agent.entrypoint"
assert_entrypoint_in_stage "${stage}" "$(manifest_gcs_entrypoint "${manifest_src}")" "gcs.entrypoint"

(cd "${stage}" && zip -qr "${archive_path}" .)

echo "wrote ${archive_path}"
echo "  runtime: rust, target: ${target}, entrypoint: ${entrypoint}"
echo "next: ADOS_SIGNING_KEY=<key> ${repo_root}/scripts/sign.sh ${archive_path}"
