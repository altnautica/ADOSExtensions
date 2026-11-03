import { describe, expect, it } from "vitest";

import type { VideoOverlayHostProps } from "@altnautica/plugin-sdk";
import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { mountOverlay } from "../src/overlay";
import { PodStateStore } from "../src/pod-state";

const PROPS: VideoOverlayHostProps = {
  droneId: "d1",
  cameraId: "main",
  streamWidth: 1920,
  streamHeight: 1080,
  renderedRect: { left: 40, top: 0, width: 640, height: 360 },
  frameTimestampMs: 1_000,
  frameAgeMs: null,
  attitude: null,
  attitudeAtFrameTime: false,
  detections: {
    frameWidth: 1920,
    frameHeight: 1080,
    frameId: 7,
    receivedAt: 1_000,
    items: [
      { bbox: { x: 0, y: 0, width: 960, height: 540 }, classLabel: "person", confidence: 0.9, trackId: 3, lockState: "locked" },
    ],
  },
};

async function mounted() {
  const harness = createPluginHarness({ grantedCapabilities: [], mount: () => undefined });
  await harness.start();
  const root = document.createElement("div");
  document.body.appendChild(root);
  const handle = mountOverlay(harness.ctx, root, new PodStateStore());
  return { harness, root, handle };
}

describe("mountOverlay", () => {
  it("sizes the canvas from the host props pushed on ctx.videoOverlay", async () => {
    const { harness, root, handle } = await mounted();
    expect(handle.hasProps()).toBe(false);

    harness.pushVideoOverlayProps(PROPS);
    await Promise.resolve();

    expect(handle.hasProps()).toBe(true);
    const canvas = root.querySelector("canvas.siyi-hud") as HTMLCanvasElement;
    expect(canvas.width).toBe(720); // left * 2 + width
    expect(canvas.height).toBe(360);
    handle.destroy();
    expect(root.querySelector("canvas")).toBeNull();
  });

  it("ignores a payload without a rendered rect", async () => {
    const { harness, handle } = await mounted();
    harness.pushEvent("video.overlay.props", { cameraId: "main" }, "ui.slot.video-overlay");
    await Promise.resolve();
    expect(handle.hasProps()).toBe(false);
    handle.destroy();
  });
});
