"""Tests for scripts/sign.sh, run through bash."""

from __future__ import annotations

import base64
import hashlib
import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import (
    BestAvailableEncryption,
    Encoding,
    NoEncryption,
    PrivateFormat,
    PublicFormat,
)

SIGN_SH = Path(__file__).resolve().parents[1] / "sign.sh"

pytestmark = pytest.mark.skipif(
    not all(shutil.which(tool) for tool in ("bash", "zip", "unzip")),
    reason="sign.sh needs bash, zip and unzip",
)

PAYLOAD = {
    "manifest.yaml": b"id: com.example.hello\nversion: 1.0.0\n",
    "gcs/plugin.bundle.js": b"export default {};\n",
}


def _posix(path: Path) -> str:
    return path.as_posix()


def _python_shim(tmp_path: Path) -> str:
    """Directory holding a `python3` that runs this interpreter (has cryptography)."""
    shim_dir = tmp_path / "shim"
    shim_dir.mkdir()
    shim = shim_dir / "python3"
    shim.write_text(f'#!/usr/bin/env bash\nexec "{Path(sys.executable).as_posix()}" "$@"\n')
    shim.chmod(0o755)
    return str(shim_dir)


def _archive(tmp_path: Path) -> Path:
    path = tmp_path / "hello.adosplug"
    with zipfile.ZipFile(path, "w") as zf:
        for name, data in PAYLOAD.items():
            zf.writestr(name, data)
    return path


def _run(tmp_path: Path, archive: Path, **env_overrides: str) -> subprocess.CompletedProcess[str]:
    env = {k: v for k, v in os.environ.items() if not k.startswith("ADOS_SIGNING")}
    env["PATH"] = _python_shim(tmp_path) + os.pathsep + env.get("PATH", "")
    env.update(env_overrides)
    return subprocess.run(
        ["bash", _posix(SIGN_SH), _posix(archive)],
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )


def _write(tmp_path: Path, name: str, data: bytes) -> str:
    path = tmp_path / name
    path.write_bytes(data)
    return _posix(path)


def _payload_hash() -> bytes:
    aggregate = hashlib.sha256()
    for name in sorted(PAYLOAD):
        aggregate.update(f"{name}\n{hashlib.sha256(PAYLOAD[name]).hexdigest()}\n".encode())
    return aggregate.digest()


def test_public_pem_exits_2(tmp_path: Path) -> None:
    pub = Ed25519PrivateKey.generate().public_key().public_bytes(
        Encoding.PEM, PublicFormat.SubjectPublicKeyInfo
    )
    result = _run(
        tmp_path,
        _archive(tmp_path),
        ADOS_SIGNING_KEY=_write(tmp_path, "signer.pub.pem", pub),
        ADOS_SIGNING_KEY_ID="test-signer",
    )
    assert result.returncode == 2
    assert "is not a PEM private key" in result.stderr
    assert not (tmp_path / "hello.signed.adosplug").exists()


def test_random_file_fails(tmp_path: Path) -> None:
    result = _run(
        tmp_path,
        _archive(tmp_path),
        ADOS_SIGNING_KEY=_write(tmp_path, "junk.bin", os.urandom(100)),
        ADOS_SIGNING_KEY_ID="test-signer",
    )
    assert result.returncode != 0
    assert not (tmp_path / "hello.signed.adosplug").exists()


def test_encrypted_pem_fails(tmp_path: Path) -> None:
    pem = Ed25519PrivateKey.generate().private_bytes(
        Encoding.PEM, PrivateFormat.PKCS8, BestAvailableEncryption(b"passphrase")
    )
    result = _run(
        tmp_path,
        _archive(tmp_path),
        ADOS_SIGNING_KEY=_write(tmp_path, "enc.priv.pem", pem),
        ADOS_SIGNING_KEY_ID="test-signer",
    )
    assert result.returncode != 0
    assert not (tmp_path / "hello.signed.adosplug").exists()


def test_missing_key_id_exits_2(tmp_path: Path) -> None:
    pem = Ed25519PrivateKey.generate().private_bytes(
        Encoding.PEM, PrivateFormat.PKCS8, NoEncryption()
    )
    result = _run(
        tmp_path,
        _archive(tmp_path),
        ADOS_SIGNING_KEY=_write(tmp_path, "signer.priv.pem", pem),
    )
    assert result.returncode == 2
    assert "ADOS_SIGNING_KEY_ID" in result.stderr


def test_raw_64_byte_key_with_wrong_public_half_exits_2(tmp_path: Path) -> None:
    seed = os.urandom(32)
    result = _run(
        tmp_path,
        _archive(tmp_path),
        ADOS_SIGNING_KEY=_write(tmp_path, "signer.ed25519", seed + os.urandom(32)),
        ADOS_SIGNING_KEY_ID="test-signer",
    )
    assert result.returncode == 2


def test_valid_private_pem_signs_and_verifies(tmp_path: Path) -> None:
    key = Ed25519PrivateKey.generate()
    pem = key.private_bytes(Encoding.PEM, PrivateFormat.PKCS8, NoEncryption())
    pub_pem = key.public_key().public_bytes(Encoding.PEM, PublicFormat.SubjectPublicKeyInfo)
    archive = _archive(tmp_path)

    result = _run(
        tmp_path,
        archive,
        ADOS_SIGNING_KEY=_write(tmp_path, "signer.priv.pem", pem),
        ADOS_SIGNING_KEY_ID="test-signer",
        ADOS_SIGNING_PUBKEY=_write(tmp_path, "signer.pub.pem", pub_pem),
    )

    assert result.returncode == 0, result.stderr
    raw_pub = key.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)
    assert hashlib.sha256(raw_pub).hexdigest()[:16] in result.stdout
    signed = tmp_path / "hello.signed.adosplug"
    with zipfile.ZipFile(signed) as zf:
        lines = zf.read("SIGNATURE").decode().splitlines()
    assert lines[0] == "test-signer"
    key.public_key().verify(base64.b64decode(lines[1]), _payload_hash())


def test_mismatched_pubkey_exits_2(tmp_path: Path) -> None:
    pem = Ed25519PrivateKey.generate().private_bytes(
        Encoding.PEM, PrivateFormat.PKCS8, NoEncryption()
    )
    other_pub = Ed25519PrivateKey.generate().public_key().public_bytes(
        Encoding.PEM, PublicFormat.SubjectPublicKeyInfo
    )
    result = _run(
        tmp_path,
        _archive(tmp_path),
        ADOS_SIGNING_KEY=_write(tmp_path, "signer.priv.pem", pem),
        ADOS_SIGNING_KEY_ID="test-signer",
        ADOS_SIGNING_PUBKEY=_write(tmp_path, "other.pub.pem", other_pub),
    )
    assert result.returncode == 2
    assert not (tmp_path / "hello.signed.adosplug").exists()
