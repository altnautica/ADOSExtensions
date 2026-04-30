#!/usr/bin/env bash
#
# Sign a packed .adosplug archive with the publisher Ed25519 key.
#
# Usage:
#   ADOS_SIGNING_KEY=/path/to/key.ed25519 scripts/sign.sh dist/<plugin>.adosplug
#
# In CI, the key arrives via repository secrets and is written to a
# tempfile. Local signing accepts either a file path or, with
# ADOS_SIGNING_KEY_INLINE=1, a base64-encoded key in ADOS_SIGNING_KEY.
#
# The signed archive carries a separate `SIGNATURE` text file with two
# lines: the signer key id, then the base64-encoded Ed25519 signature
# over the manifest body. This matches the agent's archive reader,
# which expects the SIGNATURE file alongside manifest.yaml.

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: sign.sh <archive.adosplug>" >&2
  exit 2
fi

archive_in="$1"
if [[ ! -f "${archive_in}" ]]; then
  echo "archive not found: ${archive_in}" >&2
  exit 2
fi
# Resolve to an absolute path so subsequent (cd ...) operations still
# refer to the same file.
archive="$(cd "$(dirname "${archive_in}")" && pwd)/$(basename "${archive_in}")"

if [[ -z "${ADOS_SIGNING_KEY:-}" ]]; then
  echo "ADOS_SIGNING_KEY is required (path to the Ed25519 private key)" >&2
  exit 2
fi

key_path="${ADOS_SIGNING_KEY}"
key_is_temp=0
if [[ "${ADOS_SIGNING_KEY_INLINE:-0}" == "1" ]]; then
  key_path="$(mktemp)"
  key_is_temp=1
  echo "${ADOS_SIGNING_KEY}" | base64 -d > "${key_path}"
fi

key_id="${ADOS_SIGNING_KEY_ID:-altnautica-2026-A}"

stage="$(mktemp -d)"
out="${archive%.adosplug}.signed.adosplug"
cleanup() {
  rm -rf "${stage}"
  if [[ "${key_is_temp}" == "1" ]]; then
    rm -f "${key_path}"
  fi
}
trap cleanup EXIT

unzip -qq "${archive}" -d "${stage}"

python3 - "${stage}" "${key_path}" "${key_id}" <<'PY'
import base64
import sys
from pathlib import Path

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import (
    load_pem_private_key,
)

stage = Path(sys.argv[1])
key_path = Path(sys.argv[2])
key_id = sys.argv[3]

manifest_path = stage / "manifest.yaml"
manifest_bytes = manifest_path.read_bytes()

key_bytes = key_path.read_bytes()
try:
    private = load_pem_private_key(key_bytes, password=None)
except ValueError:
    # Raw 32-byte form.
    private = Ed25519PrivateKey.from_private_bytes(key_bytes[:32])

if not isinstance(private, Ed25519PrivateKey):
    raise SystemExit("expected an Ed25519 private key")

signature = private.sign(manifest_bytes)
sig_b64 = base64.b64encode(signature).decode("ascii")

(stage / "SIGNATURE").write_text(f"{key_id}\n{sig_b64}\n")
PY

(cd "${stage}" && zip -qr "${out}" .)

echo "wrote ${out}"
