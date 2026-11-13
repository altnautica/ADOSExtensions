from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import vram  # noqa: E402


def test_parse_smi_free_reads_the_first_gpu():
    assert vram.parse_smi_free("7811\n") == 7811
    assert vram.parse_smi_free("  512 \n8000\n") == 512


def test_require_free_vram_names_the_shortfall(monkeypatch):
    monkeypatch.setattr(vram, "free_vram_mb", lambda device=0: 5200)
    with pytest.raises(vram.VramError, match="5200 MiB free, 7000 MiB needed"):
        vram.require_free_vram()


def test_require_free_vram_returns_free_when_enough(monkeypatch):
    monkeypatch.setattr(vram, "free_vram_mb", lambda device=0: 8000)
    assert vram.require_free_vram(7000) == 8000


def test_peak_is_none_without_torch(monkeypatch):
    monkeypatch.delitem(sys.modules, "torch", raising=False)
    assert vram.peak_vram_mb() is None
