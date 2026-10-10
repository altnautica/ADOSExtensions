"""Video-leg advertisement for the SIYI pod plugin.

SIYI RTSP stream layout: the pod serves exactly two concurrent streams, `main`
and `sub`, each on its own :8554 path. `main` carries the pod's primary EO
camera on every model; a multi-sensor pod (ZT6 / ZT30) serves its second sensor
on `sub`. A single-sensor pod serves only `main`. This is the layout the pod
ships with, and the plugin does not reassign it, so it is also the layout that
is on the wire.
"""

from __future__ import annotations

import logging

log = logging.getLogger(__name__)

# The advertised leg `role` uses the GCS label-map vocabulary (eo / eo_wide /
# ir); the sensor vocabulary the capability profile speaks is
# eo_zoom / eo_wide / ir.
_ROLE_FOR_SOURCE = {
    "eo_zoom": "eo",
    "eo_wide": "eo_wide",
    "ir": "ir",
}


class VideoMixin:
    """Resolves and advertises the pod's RTSP legs through ``ctx.video``.

    Mixed into :class:`~altnautica_siyi_pod.plugin.SiyiPodPlugin`, which owns
    ``_ctx``, ``_pod`` and ``_assignment``.
    """

    def _stream_layout(self) -> list[str]:
        """The pod's concurrent physical RTSP legs.

        A multi-sensor pod (ZT6 / ZT30) serves two concurrent streams —
        ``main`` and ``sub``; every other pod serves a single ``main`` leg.
        """
        profile = self._pod.profile if self._pod is not None else None
        if profile is not None and len(profile.sensors) >= 2:
            return ["main", "sub"]
        return ["main"]

    def _stream_sources(self) -> dict[str, str]:
        """Which sensor each physical leg carries.

        This is the pod's own stream layout as the vendor ships it: ``main``
        carries the primary EO camera on every model, and a multi-sensor pod
        puts its second sensor on ``sub``. The plugin does not reassign a leg's
        sensor — the opcode that does that is not a published one — so what the
        pod serves is its factory layout, and that is what gets advertised.
        """
        profile = self._pod.profile if self._pod is not None else None
        if profile is not None and len(profile.sensors) >= 2:
            secondary = "ir" if "ir" in profile.sensors else "eo_wide"
            return {"main": "eo_zoom", "sub": secondary}
        return {"main": "eo_zoom"}

    def _video_legs(self, host: str) -> list[dict]:
        """Build the pipeline's stream-source list from the pod's stream layout.

        Exactly the pod's concurrent physical legs (``main`` [+ ``sub``]), each
        pointing at its own RTSP path on the pod, with the leg ``role`` set to
        the sensor that leg carries: advertise only the streams the pod actually
        serves, never a phantom path.
        """
        legs: list[dict] = []
        sources = self._assignment
        for leg_id in self._stream_layout():
            source = sources.get(leg_id, "eo_zoom")
            legs.append(
                {
                    "id": leg_id,
                    "source": f"rtsp://{host}:8554/{leg_id}",
                    "role": _ROLE_FOR_SOURCE.get(source, "eo"),
                    "codec": "h264",
                }
            )
        return legs

    async def _configure_video(self, host: str) -> None:
        video = getattr(self._ctx, "video", None)
        if video is None or not hasattr(video, "set_source"):
            return
        cameras = self._video_legs(host)
        try:
            reply = await video.set_source(cameras)
        except Exception as exc:  # noqa: BLE001
            log.warning("video source auto-config failed: %s", exc)
            return
        # The host reports ok=False when the config saved but the pipeline
        # restart failed (streams not live). Surface it — a failed apply must be
        # visible, not silently swallowed.
        if isinstance(reply, dict) and reply.get("ok") is not True:
            log.warning("video source apply did not go live: %s", reply)
