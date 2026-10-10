"""Signing keys and env files never reach a packed .adosplug."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

PACK_EXCLUDE = Path(__file__).resolve().parents[1] / "_pack-exclude.sh"

pytestmark = pytest.mark.skipif(not shutil.which("bash"), reason="needs bash")

SECRET_PATTERNS = (".git", "*.pem", "*.key", ".env*", ".DS_Store")


def _bash(script: str) -> str:
    result = subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def _exclude_patterns() -> list[str]:
    out = _bash(
        f'source "{PACK_EXCLUDE.as_posix()}"; '
        'for a in "${PACK_RSYNC_EXCLUDES[@]}"; do printf "%s\\n" "$a"; done'
    )
    args = out.splitlines()
    return [args[i + 1] for i, arg in enumerate(args) if arg == "--exclude"]


def test_exclude_list_covers_secrets() -> None:
    patterns = _exclude_patterns()
    for pattern in SECRET_PATTERNS:
        assert pattern in patterns


@pytest.mark.skipif(not shutil.which("rsync"), reason="needs rsync")
def test_rsync_stage_drops_key_and_env(tmp_path: Path) -> None:
    src = tmp_path / "plugin"
    (src / "gcs").mkdir(parents=True)
    (src / "manifest.yaml").write_text("id: com.example.hello\n")
    (src / "gcs" / "plugin.bundle.js").write_text("export default {};\n")
    (src / "signing-key.pem").write_text("-----BEGIN PRIVATE KEY-----\nsynthetic\n")
    (src / ".env").write_text("TOKEN=synthetic\n")
    (src / "gcs" / "nested.key").write_text("synthetic\n")
    stage = tmp_path / "stage"
    stage.mkdir()

    _bash(
        f'source "{PACK_EXCLUDE.as_posix()}"; '
        f'rsync -a "${{PACK_RSYNC_EXCLUDES[@]}}" "{src.as_posix()}/" "{stage.as_posix()}/"'
    )

    staged = {p.relative_to(stage).as_posix() for p in stage.rglob("*") if p.is_file()}
    assert staged == {"manifest.yaml", "gcs/plugin.bundle.js"}
