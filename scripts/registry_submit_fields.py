"""Derive the registry submission form fields from a signed .adosplug.

Usage:
    python3 scripts/registry_submit_fields.py <archive.signed.adosplug> <manifest-out>

Writes the raw manifest.yaml text to <manifest-out> (curl sends it with
``-F manifest_yaml=<file`` so newlines survive) and prints one ``key=value``
line per remaining field. Exits 2 with a message on stderr on any error.
"""

from __future__ import annotations

import hashlib
import sys
import zipfile
from pathlib import Path

import yaml


class FieldError(Exception):
    """A submission field could not be derived."""


def canonical_payload_hash(zf: zipfile.ZipFile) -> str:
    """sha256 over sorted ``<path>\\n<sha256hex>\\n`` for every entry but SIGNATURE."""
    entries: list[tuple[str, str]] = []
    for info in zf.infolist():
        if info.is_dir() or info.filename == "SIGNATURE":
            continue
        entries.append((info.filename, hashlib.sha256(zf.read(info)).hexdigest()))
    aggregate = hashlib.sha256()
    for name, digest in sorted(entries):
        aggregate.update(f"{name}\n{digest}\n".encode("utf-8"))
    return aggregate.hexdigest()


def _clauses(spec: object) -> list[str]:
    if spec is None:
        return []
    return [part.strip() for part in str(spec).split(",") if part.strip()]


def _min_clause(spec: object) -> str:
    for clause in _clauses(spec):
        if clause.startswith(">="):
            return clause[2:].strip()
    return ""


def _max_clause(spec: object) -> str:
    for clause in _clauses(spec):
        if clause.startswith("<"):
            return clause.lstrip("<=").strip()
    return ""


def derive_fields(archive: Path) -> tuple[dict[str, str], str]:
    """Return (fields, manifest_yaml_text) for a signed archive."""
    try:
        zf = zipfile.ZipFile(archive)
    except (OSError, zipfile.BadZipFile) as exc:
        raise FieldError(f"cannot read archive {archive}: {exc}") from exc
    with zf:
        names = set(zf.namelist())
        if "SIGNATURE" not in names:
            raise FieldError("archive is unsigned; run scripts/sign.sh")
        if "manifest.yaml" not in names:
            raise FieldError("archive has no manifest.yaml")
        sig_lines = zf.read("SIGNATURE").decode("utf-8").splitlines()
        if len(sig_lines) < 2 or not sig_lines[0].strip() or not sig_lines[1].strip():
            raise FieldError("SIGNATURE must hold the signer id and the signature")
        manifest_text = zf.read("manifest.yaml").decode("utf-8")
        payload_hash = canonical_payload_hash(zf)

    try:
        manifest = yaml.safe_load(manifest_text)
    except yaml.YAMLError as exc:
        raise FieldError(f"manifest.yaml is not valid YAML: {exc}") from exc
    if not isinstance(manifest, dict):
        raise FieldError("manifest.yaml is not a mapping")
    for key in ("id", "version"):
        if not manifest.get(key):
            raise FieldError(f"manifest.yaml has no {key}")

    compat = manifest.get("compatibility") or {}
    if not isinstance(compat, dict):
        raise FieldError("manifest compatibility is not a mapping")
    agent_min = _min_clause(compat.get("ados_version"))
    if not agent_min:
        raise FieldError("manifest compatibility.ados_version has no >= clause")

    fields = {
        "plugin_id": str(manifest["id"]),
        "version": str(manifest["version"]),
        "name": str(manifest.get("name") or ""),
        "description": str(manifest.get("description") or ""),
        "license": str(manifest.get("license") or ""),
        "homepage_url": str(manifest.get("homepage") or ""),
        "signer_key_id": sig_lines[0].strip(),
        "signature": sig_lines[1].strip(),
        "payload_hash": payload_hash,
        "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "agent_min_version": agent_min,
        "agent_max_version": _max_clause(compat.get("ados_version")),
        "gcs_min_version": _min_clause(compat.get("gcs_version")),
    }
    for key, value in fields.items():
        if "\n" in value:
            raise FieldError(f"manifest field for {key} spans multiple lines")
    return fields, manifest_text


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print("usage: registry_submit_fields.py <archive> <manifest-out>", file=sys.stderr)
        return 2
    try:
        fields, manifest_text = derive_fields(Path(argv[1]))
    except FieldError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    Path(argv[2]).write_text(manifest_text, encoding="utf-8")
    for key, value in fields.items():
        print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
