"""Pod facade tests: negotiation and per-model capability gating."""

from __future__ import annotations

import pytest

from altnautica_siyi_pod import capability_profile as CP
from altnautica_siyi_pod.pod import PodUnsupported, SiyiPod
from altnautica_siyi_pod.session import SiyiSession
from altnautica_siyi_pod.transport import MockTransport
from helpers import make_pod


async def test_negotiate_resolves_each_model():
    for hw_id, model in [
        (CP.HW_A2_MINI, "A2 mini"),
        (CP.HW_A8_MINI, "A8 mini"),
        (CP.HW_ZR10, "ZR10"),
        (CP.HW_ZR30, "ZR30"),
        (CP.HW_ZT6, "ZT6"),
        (CP.HW_ZT30, "ZT30"),
    ]:
        pod, session, _t = await make_pod(model=hw_id)
        assert pod.negotiated is True
        assert pod.profile.model == model
        await session.stop()


async def test_a2_mini_gates_everything_off():
    pod, session, _t = await make_pod(model=CP.HW_A2_MINI)
    with pytest.raises(PodUnsupported):
        await pod.set_attitude(0, 0)  # fixed mount, no gimbal control
    with pytest.raises(PodUnsupported):
        await pod.set_zoom(2.0)
    with pytest.raises(PodUnsupported):
        await pod.read_laser_range()
    with pytest.raises(PodUnsupported):
        await pod.set_palette(1)
    await session.stop()


async def test_a8_mini_zoom_yes_thermal_and_laser_no():
    pod, session, transport = await make_pod(model=CP.HW_A8_MINI)
    await pod.set_zoom(3.0)  # digital zoom is allowed
    assert transport.zoom == 3.0
    with pytest.raises(PodUnsupported):
        await pod.set_palette(1)
    with pytest.raises(PodUnsupported):
        await pod.read_laser_range()
    await pod.set_attitude(10, -10)  # gimbal works
    assert transport.yaw_deg == 10.0
    await session.stop()


async def test_zt30_full_control():
    pod, session, transport = await make_pod(model=CP.HW_ZT30, laser_range_m=42.0)

    await pod.set_attitude(30, -20)
    assert transport.yaw_deg == 30.0
    assert transport.pitch_deg == -20.0

    att = await pod.read_attitude()
    assert att.yaw_deg == 30.0
    assert att.pitch_deg == -20.0

    await pod.set_zoom(10.0)
    assert transport.zoom == 10.0
    assert await pod.read_zoom() == 10.0

    assert await pod.read_laser_range() == 42.0

    await pod.set_palette(2)
    assert transport.palette == 2

    await pod.take_photo()
    assert transport.photos_taken == 1

    assert transport.recording is False
    await pod.toggle_record()
    assert transport.recording is True

    await pod.set_mode("lock")
    assert transport.gimbal_mode == "lock"

    await session.stop()


async def test_zt30_clamps_gimbal_angles():
    pod, session, transport = await make_pod(model=CP.HW_ZT30)
    await pod.set_attitude(9999, 9999)
    assert transport.yaw_deg == 360.0  # clamped to limitless-yaw bound
    assert transport.pitch_deg == 25.0  # clamped to pitch max
    await session.stop()


async def test_zt30_assigns_image_source_and_split():
    from altnautica_siyi_pod import commands as C

    pod, session, transport = await make_pod(model=CP.HW_ZT30)
    await pod.set_image_source("main", "eo_zoom")
    await pod.set_image_source("sub", "ir")
    assert transport.image_sources == {
        C.STREAM_MAIN: C.IMG_SOURCE_EO_ZOOM,
        C.STREAM_SUB: C.IMG_SOURCE_IR,
    }
    # Assigning a leg to the split source enables the on-pod composite.
    await pod.set_image_source("sub", "split")
    assert transport.split_mode is True
    assert transport.image_sources[C.STREAM_SUB] == C.IMG_SOURCE_SPLIT
    await session.stop()


async def test_read_track_box():
    pod, session, transport = await make_pod(model=CP.HW_ZT30)
    # No active track by default.
    assert await pod.read_track_box() is None
    transport.track_box = (3, 10, 20, 30, 40, True)
    box = await pod.read_track_box()
    assert box is not None
    assert box.track_id == 3
    assert box.locked is True
    assert (box.x, box.y, box.width, box.height) == (10.0, 20.0, 30.0, 40.0)
    await session.stop()


async def test_read_track_box_gated_on_ai_track():
    # The ZR10 has no on-pod tracker.
    pod, session, _t = await make_pod(model=CP.HW_ZR10)
    with pytest.raises(PodUnsupported):
        await pod.read_track_box()
    await session.stop()


async def test_negotiate_survives_unreachable_pod():
    # A pod that never answers the identity query must not raise out of
    # negotiate; it stays on the conservative fallback until it appears.
    transport = MockTransport(model=CP.HW_ZT30, answer_identity=False)
    session = SiyiSession(transport, timeout_s=0.02, retries=0)
    await session.start()
    pod = SiyiPod(session)
    profile = await pod.negotiate()  # must not raise
    assert pod.negotiated is False
    assert profile is CP.FALLBACK_PROFILE  # not a guessed model (Rule 44)
    # Once the pod answers, a re-run resolves the real model — idempotent.
    transport.answer_identity = True
    await pod.negotiate()
    assert pod.negotiated is True
    assert pod.profile.model == "ZT30"
    await session.stop()


async def test_single_eo_pod_rejects_multi_sensor_sources():
    pod, session, _t = await make_pod(model=CP.HW_A8_MINI)
    # A single-EO pod has no wide / thermal / split source to assign.
    with pytest.raises(PodUnsupported):
        await pod.set_image_source("sub", "ir")
    with pytest.raises(PodUnsupported):
        await pod.set_image_source("main", "split")
    with pytest.raises(PodUnsupported):
        await pod.set_split_mode(True)
    await session.stop()
