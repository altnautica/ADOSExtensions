"""Watchdog for the vendor VIO binary.

Tracks the time since the last ``alive`` message from the engine. If
the gap exceeds the configured grace window, the watcher invokes the
restart callback and resets its own state so the next gap is measured
from the restart, not from the original startup.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Optional

log = logging.getLogger(__name__)

DEFAULT_GRACE_S = 2.0
DEFAULT_RESTART_COOLDOWN_S = 5.0


class HeartbeatWatcher:
    """Detects a stalled VIO engine and triggers a restart.

    ``restart_cb`` is called when the engine has been silent for
    ``grace_s`` seconds AND at least ``restart_cooldown_s`` seconds
    have passed since the previous restart. The cooldown prevents a
    bad config from spawning an unbounded restart storm.
    """

    def __init__(
        self,
        *,
        restart_cb: Callable[[str], None],
        grace_s: float = DEFAULT_GRACE_S,
        restart_cooldown_s: float = DEFAULT_RESTART_COOLDOWN_S,
        clock: Optional[Callable[[], int]] = None,
    ) -> None:
        self._restart_cb = restart_cb
        self._grace_ns = int(grace_s * 1e9)
        self._cooldown_ns = int(restart_cooldown_s * 1e9)
        self._clock = clock or time.monotonic_ns
        self._last_alive_ns: Optional[int] = None
        # ``None`` means "no restart has fired yet"; the first cooldown
        # gate is therefore always open. Initialising this to 0 caused
        # the cooldown to suppress the very first restart when the
        # monotonic clock had only just rolled over, which is exactly
        # the case the watchdog is meant to act on.
        self._last_restart_ns: Optional[int] = None

    def mark_alive(self) -> None:
        """Record that the engine just sent an ``alive`` message."""

        self._last_alive_ns = self._clock()

    def check(self) -> bool:
        """Evaluate the watchdog.

        Returns True when a restart was triggered. The shim's main
        loop calls this every tick (typically the same cadence as
        :meth:`BaseVioEngine.poll`).
        """

        now = self._clock()
        if self._last_alive_ns is None:
            # No alive yet. Grace begins from first ``mark_alive`` so
            # a slow-starting binary is not killed; the shim's
            # ``handshake`` is expected to send the first alive before
            # the grace window opens.
            return False
        if now - self._last_alive_ns < self._grace_ns:
            return False
        if (
            self._last_restart_ns is not None
            and now - self._last_restart_ns < self._cooldown_ns
        ):
            return False
        elapsed_s = (now - self._last_alive_ns) / 1e9
        reason = f"heartbeat_miss_{elapsed_s:.1f}s"
        log.warning("vio engine heartbeat missed (%.1fs); restarting", elapsed_s)
        try:
            self._restart_cb(reason)
        except Exception:  # noqa: BLE001 - log + carry on
            log.exception("vio engine restart callback raised")
        self._last_restart_ns = now
        self._last_alive_ns = None
        return True
