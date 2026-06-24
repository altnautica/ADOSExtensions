import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { VideoOverlay } from "../src/VideoOverlay";
import enLocale from "../../locales/en.json";
import type { FollowState, VideoOverlayHostProps } from "../src/types";

afterEach(cleanup);

function harnessCtx() {
  return createPluginHarness({
    mount: () => undefined,
    locale: enLocale as Record<string, string>,
    grantedCapabilities: ["command.send"],
  });
}

const RECT = { left: 0, top: 0, width: 640, height: 480 };

function hostProps(): VideoOverlayHostProps {
  return {
    droneId: "drone-1",
    cameraId: "uvc-0",
    streamWidth: 640,
    streamHeight: 480,
    renderedRect: RECT,
    frameTimestampMs: 1000,
    attitude: { rollDeg: 0, pitchDeg: 0, yawDeg: 0 },
    detections: {
      frameWidth: 640,
      frameHeight: 480,
      frameId: 5,
      receivedAt: Date.now(),
      items: [
        {
          bbox: { x: 300, y: 220, width: 40, height: 40 },
          classLabel: "person",
          confidence: 0.9,
          trackId: 7,
          lockState: null,
        },
      ],
    },
  };
}

const idleFollow: FollowState = {
  active: false,
  lockState: null,
  targetId: null,
  rangeM: null,
  distanceSetpointM: null,
  heightSetpointM: null,
  commanding: false,
};

const designatingFollow: FollowState = {
  ...idleFollow,
  active: true,
};

describe("VideoOverlay", () => {
  it("renders a detection box for each detection", () => {
    const h = harnessCtx();
    render(
      <VideoOverlay ctx={h.ctx} hostProps={hostProps()} follow={idleFollow} />,
    );
    expect(screen.getAllByTestId("fm-detection-box").length).toBe(1);
  });

  it("is dormant (pointer-events off) when the skill is idle", () => {
    const h = harnessCtx();
    render(
      <VideoOverlay ctx={h.ctx} hostProps={hostProps()} follow={idleFollow} />,
    );
    const overlay = screen.getByTestId("fm-video-overlay");
    expect(overlay.getAttribute("data-designating")).toBe("false");
    expect((overlay as HTMLElement).style.pointerEvents).toBe("none");
  });

  it("shows the designate reticle when active and not yet locked", () => {
    const h = harnessCtx();
    render(
      <VideoOverlay
        ctx={h.ctx}
        hostProps={hostProps()}
        follow={designatingFollow}
      />,
    );
    expect(screen.getByTestId("fm-reticle")).toBeTruthy();
    expect(
      screen.getByTestId("fm-video-overlay").getAttribute("data-designating"),
    ).toBe("true");
  });

  it("maps a click to the box and designates it via the agent", async () => {
    const h = harnessCtx();
    const onResult = vi.fn();
    render(
      <VideoOverlay
        ctx={h.ctx}
        hostProps={hostProps()}
        follow={designatingFollow}
        onDesignateResult={onResult}
      />,
    );
    const overlay = screen.getByTestId("fm-video-overlay") as HTMLDivElement;
    // happy-dom returns a zero rect for getBoundingClientRect, so a click at
    // the box center (frame px 320,240) lands on the box (rect is the full
    // frame at 0,0). clientX/Y are wrapper-relative.
    fireEvent.click(overlay, { clientX: 320, clientY: 240 });
    // Let the command.send round-trip + the onResult continuation settle.
    await new Promise((r) => setTimeout(r, 0));

    const designate = h.calls.find(
      (c) =>
        c.method === "command.send" &&
        (c.args as { command?: string }).command === "vision.designate",
    );
    expect(designate).toBeTruthy();
    const args = (
      designate!.args as { args: { camera_id: string; track_id: number } }
    ).args;
    expect(args.camera_id).toBe("uvc-0");
    expect(args.track_id).toBe(7);
    expect(onResult).toHaveBeenCalledWith(true, expect.anything());
  });

  it("does not designate when idle even on a click", async () => {
    const h = harnessCtx();
    render(
      <VideoOverlay ctx={h.ctx} hostProps={hostProps()} follow={idleFollow} />,
    );
    const overlay = screen.getByTestId("fm-video-overlay") as HTMLDivElement;
    fireEvent.click(overlay, { clientX: 320, clientY: 240 });
    await Promise.resolve();
    const designate = h.calls.find(
      (c) =>
        c.method === "command.send" &&
        (c.args as { command?: string }).command === "vision.designate",
    );
    expect(designate).toBeUndefined();
  });
});
