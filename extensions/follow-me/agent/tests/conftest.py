"""Test setup for the Follow-Me agent plugin.

Two things are arranged here:

1. The flat ``follow_me`` package is made importable. The agent source
   lives at ``agent/follow_me`` (a flat package, the layout the plugin
   runner appends to ``sys.path`` at runtime). Tests run from the
   ``agent`` directory, so its root is added to ``sys.path``.

2. The agent SDK (``ados.plugins.manifest`` and ``ados.sdk.vision``) is
   made available. When the host agent checkout is present alongside this
   repo it is added to the path so tests exercise the real contract.
   Otherwise a minimal compatible namespace is provided so the unit tests
   for the projection geometry, the frame builders, and the lock-state
   gate run anywhere with no host checkout.
"""

from __future__ import annotations

import importlib
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import Any

_AGENT_ROOT = Path(__file__).resolve().parents[1]
if str(_AGENT_ROOT) not in sys.path:
    sys.path.insert(0, str(_AGENT_ROOT))


def _candidate_host_src_roots() -> list[Path]:
    """Plausible locations of the host agent's importable ``src`` root.

    The extensions repo normally sits beside the agent checkout in the
    same monorepo, so a few relative hops cover the common layouts. Only
    existing directories that actually contain the ``ados`` package are
    returned.
    """
    here = Path(__file__).resolve()
    roots: list[Path] = []
    for parent in here.parents:
        roots.append(parent / "ADOSDroneAgent" / "src")
        roots.append(parent.parent / "ADOSDroneAgent" / "src")
    seen: set[Path] = set()
    out: list[Path] = []
    for root in roots:
        if root in seen:
            continue
        seen.add(root)
        if (root / "ados" / "__init__.py").is_file():
            out.append(root)
    return out


def _try_real_host_sdk() -> bool:
    """Add the real host agent ``src`` to the path if it is reachable.

    Returns True when ``ados.plugins.manifest`` and ``ados.sdk.vision``
    import from the real package.
    """
    for root in _candidate_host_src_roots():
        entry = str(root)
        if entry not in sys.path:
            sys.path.insert(0, entry)
        try:
            importlib.import_module("ados.plugins.manifest")
            importlib.import_module("ados.sdk.vision")
            return True
        except Exception:  # noqa: BLE001
            # Remove the speculative entry and try the next layout.
            if entry in sys.path:
                sys.path.remove(entry)
    return False


def _install_sdk_stub() -> None:
    """Provide a minimal ``ados.plugins.manifest`` + ``ados.sdk.vision``.

    Only the surface the plugin imports at module load and the surface the
    tests construct is stubbed: the manifest dataclasses (free of the
    host's pydantic validators) and the vision wire shapes
    (``BoundingBox`` / ``Detection`` / ``DetectionBatch``). Behaviour
    matches the real classes for the fields these tests touch.
    """
    ados = ModuleType("ados")
    ados.__path__ = []  # type: ignore[attr-defined]
    plugins = ModuleType("ados.plugins")
    plugins.__path__ = []  # type: ignore[attr-defined]
    manifest = ModuleType("ados.plugins.manifest")
    sdk = ModuleType("ados.sdk")
    sdk.__path__ = []  # type: ignore[attr-defined]
    vision = ModuleType("ados.sdk.vision")

    @dataclass
    class AgentBlock:
        entrypoint: str
        isolation: str = "subprocess"
        runtime: str = "python"
        permissions: list[Any] = field(default_factory=list)

    @dataclass
    class Compatibility:
        ados_version: str
        gcs_version: str | None = None

    @dataclass
    class PluginManifest:
        schema_version: int = 1
        id: str = ""
        version: str = "0.0.0"
        name: str = ""
        description: str = ""
        author: str = ""
        license: str = ""
        risk: str = "low"
        compatibility: Any = None
        agent: Any = None

    manifest.AgentBlock = AgentBlock
    manifest.Compatibility = Compatibility
    manifest.PluginManifest = PluginManifest

    @dataclass(frozen=True)
    class BoundingBox:
        x: float
        y: float
        width: float
        height: float

        def to_dict(self) -> dict[str, Any]:
            return {
                "x": float(self.x),
                "y": float(self.y),
                "width": float(self.width),
                "height": float(self.height),
            }

        @classmethod
        def from_dict(cls, raw: dict[str, Any]) -> "BoundingBox":
            return cls(
                x=float(raw["x"]),
                y=float(raw["y"]),
                width=float(raw["width"]),
                height=float(raw["height"]),
            )

    @dataclass(frozen=True)
    class Detection:
        bbox: BoundingBox
        class_label: str
        confidence: float
        track_id: int | None = None
        assoc_confidence: float | None = None
        lock_state: str | None = None

    @dataclass(frozen=True)
    class DetectionBatch:
        model_id: str
        camera_id: str
        frame_id: int
        ts_ms: int
        detections: list[Detection] = field(default_factory=list)
        frame_width: int | None = None
        frame_height: int | None = None

    vision.BoundingBox = BoundingBox
    vision.Detection = Detection
    vision.DetectionBatch = DetectionBatch

    sys.modules["ados"] = ados
    sys.modules["ados.plugins"] = plugins
    sys.modules["ados.plugins.manifest"] = manifest
    sys.modules["ados.sdk"] = sdk
    sys.modules["ados.sdk.vision"] = vision


if not _try_real_host_sdk():
    _install_sdk_stub()
