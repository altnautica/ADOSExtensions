"""Tests for scripts/registry_submit_fields.py."""

from __future__ import annotations

import base64
import hashlib
import importlib.util
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

HELPER = Path(__file__).resolve().parents[1] / "registry_submit_fields.py"

MANIFEST = """\
schema_version: 2
id: com.example.hello
name: "Hello"
version: "1.2.3"
description: "Synthetic test plugin"
license: "GPL-3.0-or-later"
homepage: "https://example.com/hello"
compatibility:
  ados_version: ">=0.99.125,<1.0.0"
  gcs_version: ">=0.40.0"
"""

PAYLOAD = {
    "manifest.yaml": MANIFEST.encode("utf-8"),
    "gcs/plugin.bundle.js": b"export default {};\n",
    "locales/en.json": b"{}\n",
}


def _load_helper():
    spec = importlib.util.spec_from_file_location("registry_submit_fields", HELPER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _expected_hash() -> str:
    aggregate = hashlib.sha256()
    for name in sorted(PAYLOAD):
        aggregate.update(f"{name}\n{hashlib.sha256(PAYLOAD[name]).hexdigest()}\n".encode())
    return aggregate.hexdigest()


def _build(path: Path, *, signed: bool) -> tuple[str, str]:
    key = Ed25519PrivateKey.generate()
    signature = base64.b64encode(key.sign(bytes.fromhex(_expected_hash()))).decode()
    with zipfile.ZipFile(path, "w") as zf:
        for name, data in PAYLOAD.items():
            zf.writestr(name, data)
        if signed:
            zf.writestr("SIGNATURE", f"test-signer\n{signature}\n")
    return "test-signer", signature


def _run(archive: Path, manifest_out: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(HELPER), str(archive), str(manifest_out)],
        capture_output=True,
        text=True,
        check=False,
    )


def test_signed_archive_yields_every_field(tmp_path: Path) -> None:
    archive = tmp_path / "hello.signed.adosplug"
    signer, signature = _build(archive, signed=True)
    manifest_out = tmp_path / "manifest.yaml"

    result = _run(archive, manifest_out)

    assert result.returncode == 0, result.stderr
    fields = dict(line.split("=", 1) for line in result.stdout.splitlines())
    assert fields == {
        "plugin_id": "com.example.hello",
        "version": "1.2.3",
        "name": "Hello",
        "description": "Synthetic test plugin",
        "license": "GPL-3.0-or-later",
        "homepage_url": "https://example.com/hello",
        "signer_key_id": signer,
        "signature": signature,
        "payload_hash": _expected_hash(),
        "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "agent_min_version": "0.99.125",
        "agent_max_version": "1.0.0",
        "gcs_min_version": "0.40.0",
    }
    assert manifest_out.read_text(encoding="utf-8") == MANIFEST


def test_payload_hash_matches_canonical_form(tmp_path: Path) -> None:
    archive = tmp_path / "hello.signed.adosplug"
    _build(archive, signed=True)
    helper = _load_helper()
    with zipfile.ZipFile(archive) as zf:
        assert helper.canonical_payload_hash(zf) == _expected_hash()


def test_unsigned_archive_exits_2(tmp_path: Path) -> None:
    archive = tmp_path / "hello.adosplug"
    _build(archive, signed=False)

    result = _run(archive, tmp_path / "manifest.yaml")

    assert result.returncode == 2
    assert "archive is unsigned; run scripts/sign.sh" in result.stderr


@pytest.mark.parametrize(
    ("spec", "minimum", "maximum"),
    [
        (">=0.99.125,<1.0.0", "0.99.125", "1.0.0"),
        (">=1.0.0, <=1.4.0", "1.0.0", "1.4.0"),
        (">=2.0.0", "2.0.0", ""),
    ],
)
def test_version_clause_parsing(spec: str, minimum: str, maximum: str) -> None:
    helper = _load_helper()
    assert helper._min_clause(spec) == minimum
    assert helper._max_clause(spec) == maximum


def test_missing_min_version_exits_2(tmp_path: Path) -> None:
    archive = tmp_path / "hello.signed.adosplug"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("manifest.yaml", MANIFEST.replace(">=0.99.125,", ""))
        zf.writestr("SIGNATURE", "test-signer\nAAAA\n")

    result = _run(archive, tmp_path / "manifest.yaml")

    assert result.returncode == 2
    assert "ados_version" in result.stderr
