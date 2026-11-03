import { describe, expect, it } from "vitest";

import { isVideoOverlayHostProps, type VideoOverlayHostProps } from "../src/api";
import { createPluginHarness } from "../src/harness";

function sampleProps(): VideoOverlayHostProps {
  return {
    droneId: "local",
    cameraId: "main",
    streamWidth: 1280,
    streamHeight: 720,
    renderedRect: { left: 0, top: 37.5, width: 800, height: 450 },
    frameTimestampMs: 1_700_000_000_000,
    frameAgeMs: null,
    attitude: { rollDeg: 10, pitchDeg: -5, yawDeg: 90 },
    attitudeAtFrameTime: false,
    detections: {
      frameWidth: 1280,
      frameHeight: 720,
      frameId: 41,
      receivedAt: 1_700_000_000_000,
      items: [
        {
          bbox: { x: 10, y: 20, width: 30, height: 40 },
          classLabel: "person",
          confidence: 0.9,
          trackId: 7,
          lockState: "locked",
        },
      ],
    },
  };
}

describe("ctx.videoOverlay", () => {
  it("delivers pushed host props and stops after unsubscribe", async () => {
    const seen: VideoOverlayHostProps[] = [];
    let off: (() => void) | undefined;
    const harness = createPluginHarness({
      mount: async (ctx) => {
        off = ctx.videoOverlay.onProps((p) => seen.push(p));
      },
    });
    await harness.start();

    harness.pushVideoOverlayProps(sampleProps());
    expect(seen).toHaveLength(1);
    expect(seen[0]?.renderedRect.top).toBe(37.5);
    expect(seen[0]?.detections?.items[0]?.trackId).toBe(7);

    // A stale batch arrives as null detections, an unknown attitude as null.
    harness.pushVideoOverlayProps({ ...sampleProps(), detections: null, attitude: null });
    expect(seen[1]?.detections).toBeNull();
    expect(seen[1]?.attitude).toBeNull();

    off?.();
    harness.pushVideoOverlayProps(sampleProps());
    expect(seen).toHaveLength(2);

    await harness.teardown();
  });

  it("drops a malformed payload", async () => {
    const seen: unknown[] = [];
    const harness = createPluginHarness({
      mount: async (ctx) => {
        ctx.videoOverlay.onProps((p) => seen.push(p));
      },
    });
    await harness.start();
    harness.pushEvent("video.overlay.props", { cameraId: "main" }, "ui.slot.video-overlay");
    harness.pushEvent("video.overlay.props", null, "ui.slot.video-overlay");
    expect(seen).toHaveLength(0);
    await harness.teardown();
  });

  it("checks the payload shape", () => {
    expect(isVideoOverlayHostProps(sampleProps())).toBe(true);
    expect(isVideoOverlayHostProps({ ...sampleProps(), detections: null })).toBe(true);
    expect(isVideoOverlayHostProps({ ...sampleProps(), renderedRect: { left: 0 } })).toBe(false);
    expect(isVideoOverlayHostProps({ ...sampleProps(), detections: { items: "x" } })).toBe(false);
    expect(isVideoOverlayHostProps("props")).toBe(false);
  });
});
