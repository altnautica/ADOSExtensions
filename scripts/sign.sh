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
# The signed manifest carries the signature inside `signing.signature`
# and the signer key id in `signing.signer_key_id`. The signed archive
# is written next to the input as <name>.signed.adosplug.

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: sign.sh <archive.adosplug>" >&2
  exit 2
fi

archive="$1"
if [[ ! -f "${archive}" ]]; then
  echo "archive not found: ${archive}" >&2
  exit 2
fi

if [[ -z "${ADOS_SIGNING_KEY:-}" ]]; then
  echo "ADOS_SIGNING_KEY is required (path to the Ed25519 private key)" >&2
  exit 2
fi

key_path="${ADOS_SIGNING_KEY}"
if [[ "${ADOS_SIGNING_KEY_INLINE:-0}" == "1" ]]; then
  key_path="$(mktemp)"
  echo "${ADOS_SIGNING_KEY}" | base64 -d > "${key_path}"
  trap 'rm -f "${key_path}"' EXIT
fi

key_id="${ADOS_SIGNING_KEY_ID:-altnautica-2026-A}"

stage="$(mktemp -d)"
out="${archive%.adosplug}.signed.adosplug"
trap 'rm -rf "${stage}"; rm -f "${key_path:-}"' EXIT

# Unpack, sign the manifest body, repack.
unzip -qq "${archive}" -d "${stage}"

python3 - "${stage}" "${key_path}" "${key_id}" <<'PY'
import base64
import re
import sys
from pathlib import Path

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import (
    load_pem_private_key,
)

stage = Path(sys.argv[1])
key_path = Path(sys.argv[2])
key_id = sys.argv[3]

manifest = stage / "manifest.yaml"
raw = manifest.read_text()

# Strip the existing signature line(s) before signing so the signature
# covers only the canonical manifest body.
canonical = re.sub(
    r'^  signature:[^\n]*\n',
    '  signature: ""\n',
    raw,
    flags=re.MULTILINE,
)

key_bytes = key_path.read_bytes()
try:
    private = load_pem_private_key(key_bytes, password=None)
except ValueError:
    # Raw 32-byte form
    private = Ed25519PrivateKey.from_private_bytes(key_bytes[:32])

if not isinstance(private, Ed25519PrivateKey):
    raise SystemExit("expected an Ed25519 private key")

signature = private.sign(canonical.encode("utf-8"))
sig_b64 = base64.b64encode(signature).decode("ascii")

new_text = re.sub(
    r'^(  signature:\s*)"[^"]*"',
    f'\\1"{sig_b64}"',
    canonical,
    count=1,
    flags=re.MULTILINE,
)
new_text = re.sub(
    r'^(  signer_key_id:\s*)"[^"]*"',
    f'\\1"{key_id}"',
    new_text,
    count=1,
    flags=re.MULTILINE,
)
manifest.write_text(new_text)
PY

(cd "${stage}" && zip -qr "${out}" .)

echo "wrote ${out}"
