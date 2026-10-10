#!/usr/bin/env bash
#
# Sign a packed .adosplug archive with the publisher Ed25519 key.
#
# Usage:
#   ADOS_SIGNING_KEY=/path/to/<signer-id>.priv.pem \
#   ADOS_SIGNING_KEY_ID=<signer-id> \
#     scripts/sign.sh dist/<plugin>.adosplug
#
# Required env:
#   ADOS_SIGNING_KEY     Path to the Ed25519 private key: the unencrypted
#                        PKCS#8 <signer-id>.priv.pem written by
#                        `ados plugin keygen`, or a raw 32-byte seed /
#                        64-byte seed+public key file.
#   ADOS_SIGNING_KEY_ID  Signer id the agent looks up the public key under.
#                        There is no default.
# Optional env:
#   ADOS_SIGNING_KEY_INLINE=1  ADOS_SIGNING_KEY holds base64 key material
#                              instead of a path (CI secrets).
#   ADOS_SIGNING_PUBKEY        Path to the matching public key PEM. When set,
#                              the signature is verified after writing and a
#                              mismatch exits 2.
#
# The signed archive carries a separate `SIGNATURE` text file with two
# lines: the signer key id, then the base64-encoded Ed25519 signature
# over the canonical payload hash. The payload hash is sha256 over the
# sorted concatenation of `<path>\n<sha256-hex>\n` for every archive
# entry except SIGNATURE itself. This matches the agent's archive
# reader at ados.plugins.archive._canonical_payload_hash.

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: ADOS_SIGNING_KEY=<key> ADOS_SIGNING_KEY_ID=<signer-id> sign.sh <archive.adosplug>" >&2
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
if [[ -z "${ADOS_SIGNING_KEY_ID:-}" ]]; then
  echo "ADOS_SIGNING_KEY_ID is required (the signer id your public key is enrolled under)" >&2
  exit 2
fi
key_id="${ADOS_SIGNING_KEY_ID}"
pubkey_path="${ADOS_SIGNING_PUBKEY:-}"
if [[ -n "${pubkey_path}" && ! -f "${pubkey_path}" ]]; then
  echo "ADOS_SIGNING_PUBKEY not found: ${pubkey_path}" >&2
  exit 2
fi

key_path="${ADOS_SIGNING_KEY}"
key_is_temp=0
if [[ "${ADOS_SIGNING_KEY_INLINE:-0}" == "1" ]]; then
  key_path="$(mktemp)"
  key_is_temp=1
  echo "${ADOS_SIGNING_KEY}" | base64 -d > "${key_path}"
elif [[ ! -f "${key_path}" ]]; then
  echo "ADOS_SIGNING_KEY not found: ${key_path}" >&2
  exit 2
fi

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

python3 - "${stage}" "${key_path}" "${key_id}" "${pubkey_path}" <<'PY'
import base64
import hashlib
import sys
from pathlib import Path

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey,
    Ed25519PublicKey,
)
from cryptography.hazmat.primitives.serialization import (
    Encoding,
    PublicFormat,
    load_pem_private_key,
    load_pem_public_key,
)

stage = Path(sys.argv[1])
key_path = Path(sys.argv[2])
key_id = sys.argv[3]
pubkey_path = sys.argv[4]


def fail(msg: str) -> None:
    print(msg, file=sys.stderr)
    raise SystemExit(2)


def load_private(raw: bytes) -> Ed25519PrivateKey:
    if raw.lstrip().startswith(b"-----BEGIN"):
        hint = (
            f"{key_path} is not a PEM private key; pass the "
            "<signer-id>.priv.pem written by `ados plugin keygen`"
        )
        try:
            key = load_pem_private_key(raw, password=None)
        except TypeError:
            fail(f"{hint} (the key is encrypted; export it unencrypted)")
        except ValueError:
            fail(hint)
        if not isinstance(key, Ed25519PrivateKey):
            fail(f"{key_path} is not an Ed25519 private key")
        return key
    if len(raw) == 32:
        return Ed25519PrivateKey.from_private_bytes(raw)
    if len(raw) == 64:
        key = Ed25519PrivateKey.from_private_bytes(raw[:32])
        derived = key.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)
        if derived != raw[32:]:
            fail(f"{key_path}: bytes 32-63 do not match the public key derived from the seed")
        return key
    fail(
        f"{key_path} is not a PEM private key or a raw 32/64-byte Ed25519 key "
        f"({len(raw)} bytes); pass the <signer-id>.priv.pem written by `ados plugin keygen`"
    )
    raise AssertionError("unreachable")


private = load_private(key_path.read_bytes())
public_raw = private.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)

# Compute the canonical payload hash the agent verifies against:
# sort entries by archive path (POSIX), exclude the SIGNATURE file
# itself, concatenate `<path>\n<sha256-hex>\n` for each, then sha256
# the concatenation. Signs the 32-byte digest, not the manifest body.
files: list[tuple[str, bytes]] = []
for path in sorted(stage.rglob("*")):
    if not path.is_file():
        continue
    rel = path.relative_to(stage).as_posix()
    if rel == "SIGNATURE":
        continue
    files.append((rel, path.read_bytes()))

aggregate = hashlib.sha256()
for rel, data in sorted(files, key=lambda item: item[0]):
    digest = hashlib.sha256(data).hexdigest()
    aggregate.update(f"{rel}\n{digest}\n".encode("utf-8"))
payload_hash = aggregate.digest()

signature = private.sign(payload_hash)
sig_b64 = base64.b64encode(signature).decode("ascii")

(stage / "SIGNATURE").write_text(f"{key_id}\n{sig_b64}\n")

print(f"signer {key_id} fingerprint {hashlib.sha256(public_raw).hexdigest()[:16]}")

if pubkey_path:
    try:
        expected = load_pem_public_key(Path(pubkey_path).read_bytes())
    except ValueError:
        fail(f"{pubkey_path} is not a PEM public key")
    if not isinstance(expected, Ed25519PublicKey):
        fail(f"{pubkey_path} is not an Ed25519 public key")
    written = (stage / "SIGNATURE").read_text().splitlines()
    try:
        expected.verify(base64.b64decode(written[1]), payload_hash)
    except InvalidSignature:
        fail(f"signature does not verify against {pubkey_path}; the private key does not match it")
    print(f"verified against {pubkey_path}")
PY

rm -f "${out}"
(cd "${stage}" && zip -qr "${out}" .)

echo "wrote ${out}"
