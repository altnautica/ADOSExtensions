"""Host-profile probe for the vision-nav plugin.

The agent ships in two flavours on the same install path: a drone
profile that talks to a flight controller and a ground-station profile
that pairs an air-link receiver with a ground display. Vision
navigation is drone-only. Installing the plugin on a ground-station
host is a usage error; the plugin refuses to enable rather than
silently consuming a video device that the ground station might need.

The canonical profile source lives in the agent at
``/etc/ados/profile.conf`` and reads ``profile = drone`` or
``profile = ground-station``. The probe falls back to the
``ADOS_PROFILE`` env var when the file is absent (the case during
unit tests + during initial install before the profile is written).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

PROFILE_CONF = Path("/etc/ados/profile.conf")
ENV_VAR = "ADOS_PROFILE"


@dataclass(frozen=True)
class HostProfile:
    """The detected host profile.

    ``profile`` is one of ``"drone"``, ``"ground-station"``, or
    ``"unknown"``. The plugin treats ``unknown`` as "warn and continue
    in degraded mode" so a fresh-install host without a profile file
    still gets to load the plugin.
    """

    profile: str
    source: str  # "file" | "env" | "default"


def detect_host_profile(
    conf_path: Path = PROFILE_CONF,
    env_var: str = ENV_VAR,
) -> HostProfile:
    """Read the host profile from the canonical sources.

    Order: file, env var, default. The file is the authoritative
    source on a real install. The env var lets unit tests override
    without writing to ``/etc``. The default returns ``"unknown"`` so
    the plugin can log a warning but still proceed (refusing on
    unknown would brick a brand-new install).
    """

    try:
        text = conf_path.read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, OSError):
        text = ""

    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        if key.strip().lower() == "profile":
            return HostProfile(profile=value.strip().lower(), source="file")

    env_value = os.environ.get(env_var, "").strip().lower()
    if env_value:
        return HostProfile(profile=env_value, source="env")

    return HostProfile(profile="unknown", source="default")


def is_drone_profile(profile: HostProfile) -> bool:
    """Whether this host should run the plugin.

    Returns ``True`` for ``drone`` and ``unknown`` (the latter so a
    fresh-install host without a profile file still gets to try the
    plugin and the GCS can surface a warning). Returns ``False`` for
    any explicit ground-station variant.
    """

    p = profile.profile
    if p == "drone":
        return True
    if p == "unknown":
        return True
    # Match all ground-station variants: ground-station,
    # ground-station-direct, ground-station-relay, ground-station-receiver.
    if p.startswith("ground-station"):
        return False
    # Treat anything else as drone by default so unrecognised profile
    # strings do not lock out an otherwise-working host.
    return True
