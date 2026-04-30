#!/usr/bin/env bash
#
# Pack a built extension into a .adosplug archive.
#
# Usage:
#   scripts/pack.sh <extension-folder-or-path>
#
# Examples:
#   scripts/pack.sh battery-health-panel        # resolves to extensions/battery-health-panel
#   scripts/pack.sh ./my-plugin                 # any directory containing manifest.yaml
#   scripts/pack.sh /abs/path/to/my-plugin      # absolute path also accepted
#
# The script:
#   1. Builds the GCS half (esbuild -> plugin.bundle.js)
#   2. Computes SHA-256 hashes for every asset
#   3. Replaces the <computed-by-pack.sh> placeholders in manifest.yaml
#   4. Zips everything into dist/<plugin-id>-<version>.adosplug
#
# Signing is a separate step: scripts/sign.sh <archive>.

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: pack.sh <extension-folder-or-path>" >&2
  exit 2
fi

arg="$1"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"

# Resolve the extension directory. If the argument is a path to an existing
# directory containing a manifest.yaml, use it directly. Otherwise treat it
# as a folder name under the monorepo's extensions/ tree.
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

# Build the GCS bundle if there is one.
if [[ -d "${ext_dir}/gcs" ]]; then
  echo "building gcs bundle..."
  (cd "${ext_dir}/gcs" && pnpm build)
fi

# Read plugin id and version from the manifest.
plugin_id="$(grep -E '^  id:' "${manifest_src}" | head -n1 | sed -E 's/.*id:[[:space:]]*//; s/[[:space:]]*$//')"
plugin_version="$(grep -E '^  version:' "${manifest_src}" | head -n1 | sed -E 's/.*version:[[:space:]]*"?([^"]*)"?/\1/')"

if [[ -z "${plugin_id}" || -z "${plugin_version}" ]]; then
  echo "could not parse plugin id/version from manifest" >&2
  exit 1
fi

archive_name="${plugin_id}-${plugin_version}.adosplug"
dist_dir="${ext_dir}/dist"
mkdir -p "${dist_dir}"
archive_path="${dist_dir}/${archive_name}"
rm -f "${archive_path}"

stage="$(mktemp -d)"
trap 'rm -rf "${stage}"' EXIT

# Copy the extension contents into the staging area, excluding build
# artifacts and the dist folder we just created.
rsync -a \
  --exclude 'dist' \
  --exclude 'node_modules' \
  --exclude '.venv' \
  --exclude '.pnpm-store' \
  --exclude 'tests' \
  --exclude '__pycache__' \
  --exclude '.tsbuildinfo' \
  --exclude 'tsconfig.json' \
  --exclude 'src' \
  --exclude 'package.json' \
  "${ext_dir}/" "${stage}/"

# Re-write asset hashes inside manifest.yaml using sha256 of the staged files.
manifest_dest="${stage}/manifest.yaml"
python3 - "${stage}" "${manifest_dest}" <<'PY'
import hashlib
import re
import sys
from pathlib import Path

stage = Path(sys.argv[1])
manifest = Path(sys.argv[2])
text = manifest.read_text()

def hash_for(rel: str) -> str:
    p = stage / rel
    if not p.exists():
        raise SystemExit(f"asset listed in manifest but not staged: {rel}")
    return hashlib.sha256(p.read_bytes()).hexdigest()

# Match: - path: "<path>"\n    role: "..."\n    sha256: "<computed-by-pack.sh>"
pattern = re.compile(
    r'(- path:\s*"(?P<path>[^"]+)"\n\s+role:\s*"[^"]+"\n\s+sha256:\s*")<computed-by-pack\.sh>(")',
)

def repl(match: re.Match[str]) -> str:
    rel = match.group("path")
    # Group 1 = full prefix up to and including the opening quote of the
    # sha256 value. Group 2 = the named `path` (inner). Group 3 = the
    # closing quote after the placeholder.
    return f'{match.group(1)}{hash_for(rel)}{match.group(3)}'

new_text, n = pattern.subn(repl, text)
if n == 0:
    print("warning: no asset hashes were rewritten", file=sys.stderr)
manifest.write_text(new_text)
PY

(cd "${stage}" && zip -qr "${archive_path}" .)

echo "wrote ${archive_path}"
