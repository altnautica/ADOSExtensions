"""Plugin entry point.

The supervisor imports ``GimbalV2Plugin`` and calls ``setup()`` once.
The plugin selects a driver based on the ``transport`` config field,
opens a session, registers the gimbal manager component, and returns
control. Teardown closes the session in reverse.

This module stays small on purpose; the heavy lifting is in
``mavlink_driver.py``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Awaitable, Callable

from altnautica_gimbal_v2.gremsy_driver import GremsyGimbalDriver
from altnautica_gimbal_v2.mavlink_driver import MavlinkGimbalDriver, RouterHandle
from altnautica_gimbal_v2.sbgc_driver import SimpleBgcGimbalDriver
from altnautica_gimbal_v2.storm32_driver import Storm32NtGimbalDriver

log = logging.getLogger(__name__)


_DRIVER_FACTORIES: dict[str, Callable[[Any], Any]] = {
    "mavlink": lambda router: MavlinkGimbalDriver(router=router),
    "sbgc-uart": lambda _router: SimpleBgcGimbalDriver(),
    "storm32-uart": lambda _router: Storm32NtGimbalDriver(),
    "gremsy-uart": lambda _router: GremsyGimbalDriver(),
}


@dataclass
class GimbalV2Plugin:
    """Top-level plugin object loaded by the agent supervisor.

    The supervisor builds an instance with the host-supplied context
    and calls ``setup()``. The plugin reads its config, selects a
    driver, opens a session, and returns. Telemetry pumping happens
    inside the driver's own coroutines.
    """

    config: dict[str, Any]
    router: RouterHandle
    publish_event: Callable[[str, dict[str, Any]], Awaitable[None]]

    def __post_init__(self) -> None:
        self._driver: Any = None
        self._session: Any = None

    async def setup(self) -> None:
        transport = str(self.config.get("transport", "mavlink"))
        factory = _DRIVER_FACTORIES.get(transport)
        if factory is None:
            raise ValueError(
                f"unknown gimbal transport: {transport!r}; "
                f"expected one of {sorted(_DRIVER_FACTORIES)}"
            )
        self._driver = factory(self.router)
        candidates = await self._driver.discover()
        if not candidates:
            log.warning("gimbal driver %s reported no candidates", transport)
            return
        self._session = await self._driver.open(candidates[0], self.config)
        await self.publish_event(
            "sensor.gimbal.health",
            {"transport": transport, "responsive": True},
        )

    async def teardown(self) -> None:
        if self._session is not None and self._driver is not None:
            await self._driver.close(self._session)
        self._session = None
        self._driver = None

    @property
    def driver(self) -> Any:
        return self._driver

    @property
    def session(self) -> Any:
        return self._session
