"""Plugin entry point for the ADOS Thermal Camera FLIR Lepton USB UVC.

The agent half ships a single class, :class:`ThermalUsbPlugin`, that
the host instantiates from a subprocess hosting environment. The host
calls :meth:`on_start` after capability tokens are issued and
:meth:`on_stop` when the plugin is torn down.

The plugin's job at start is small:

1. Construct a UVC backend. In v1.0 the production binding is not in
   tree so the default is the in-tree :class:`MockUvcBackend`. The
   agent passes a real backend factory via ``ctx.hardware.uvc.backend``
   when the libuvc binding lands.

2. Build a :class:`LeptonUvcDriver` against the backend.

3. Register the driver with the peripheral manager via
   ``ctx.peripheral_manager.register_camera_driver(driver)``.

The peripheral manager handles discovery, arbitration, opening
sessions, and pumping frames onto the event bus. The plugin does not
need to subscribe to telemetry or attitude in v1.0 (the host's
recording subsystem and the GCS overlay handle those concerns through
the existing telemetry path).

Spec references that justify this surface:

* ``02-architecture.md`` section 2: agent-half component map and
  driver registration.
* ``03-dependencies.md`` section 2: ``sensor.camera.register``
  permission and the peripheral manager registry contract.
* ``05-design.md`` sections 3 and 4: palette and spot-meter wiring at
  the driver layer.
"""

from __future__ import annotations

from typing import Any, Callable, Protocol

from altnautica_thermal_camera.driver import LeptonUvcDriver
from altnautica_thermal_camera.uvc_backend import LibUvcBackend, MockUvcBackend


class _PeripheralManager(Protocol):
    """The slice of the peripheral manager the plugin depends on.

    The host injects an object that conforms to this Protocol via
    ``ctx.peripheral_manager``. v1.0 only needs camera registration;
    a future revision may add hooks for power-management and hot-plug
    events.
    """

    def register_camera_driver(self, driver: Any) -> None: ...

    def unregister_camera_driver(self, driver: Any) -> None: ...


class _PluginContext(Protocol):
    """The plugin host context shape the entry point reads.

    The host provides this object at ``on_start``. The plugin only
    touches ``peripheral_manager`` and ``log`` in v1.0; richer
    subscriptions (events, telemetry, recording) are wired by the
    GCS half and the host's video pipeline.
    """

    peripheral_manager: _PeripheralManager
    log: Any


BackendFactory = Callable[[], LibUvcBackend]


class ThermalUsbPlugin:
    """Entry point for the thermal camera plugin.

    The plugin is constructed by the host. Optional ``backend_factory``
    is the seam by which a production agent injects a real libuvc
    binding. When unset, a :class:`MockUvcBackend` is used so the
    plugin starts cleanly on a bench rig with no PureThermal hardware
    plugged in (frames stream synthetic data; the GCS overlay paints
    them as if they were real). When the libuvc binding lands the
    host swaps the factory; the rest of the plugin does not change.
    """

    plugin_id = "com.altnautica.thermal-flir-lepton-usb"
    version = "1.0.0"

    def __init__(self, backend_factory: BackendFactory | None = None) -> None:
        self._backend_factory: BackendFactory = (
            backend_factory if backend_factory is not None else MockUvcBackend
        )
        self._driver: LeptonUvcDriver | None = None
        self._registered = False

    async def on_start(self, ctx: _PluginContext) -> None:
        """Build the driver and register it with the peripheral manager."""

        backend = self._backend_factory()
        self._driver = LeptonUvcDriver(backend)
        ctx.peripheral_manager.register_camera_driver(self._driver)
        self._registered = True
        try:
            ctx.log.info(
                "thermal-flir-lepton-usb registered camera driver",
                extra={"driver_id": self._driver.driver_id},
            )
        except Exception:
            # Logging is best-effort. Production contexts always provide a
            # structured logger; tests pass a stub.
            pass

    async def on_stop(self, ctx: _PluginContext) -> None:
        """Unregister the driver and release the backend."""

        if self._driver is not None and self._registered:
            try:
                ctx.peripheral_manager.unregister_camera_driver(self._driver)
            finally:
                self._registered = False
        self._driver = None

    @property
    def driver(self) -> LeptonUvcDriver | None:
        """Test helper: expose the driver instance after start."""

        return self._driver
