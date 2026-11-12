/**
 * The Live World cards say when the capture produces no world model (a link
 * that carries no keyframes, a session still waiting for its position anchor,
 * a capped count, dropped keyframes), show every agent word through the
 * catalogue, and name the compute node instead of printing its device id.
 */

import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import messages from "../../../locales/en.json";
import { LiveWorldCards } from "../../src/pages/drone/LiveWorldCards";
import { EMPTY_ATLAS_LIVE, type AtlasLiveState } from "../../src/stores/atlas-store";

afterEach(cleanup);

const c = messages.atlas.capture;

function show(patch: Partial<AtlasLiveState>, nodes: { deviceId: string; name: string }[] = []) {
  render(
    <LiveWorldCards
      live={{ ...EMPTY_ATLAS_LIVE, state: "capturing", updatedAt: 1, ...patch }}
      stale={false}
      lastKeyframeAgo="1s ago"
      nodes={nodes}
    />,
  );
}

describe("LiveWorldCards", () => {
  it("flags a link that carries no keyframes, with the reason", () => {
    show({ bearer: "wfb-relay", keyframesCarried: false, degradedReason: "relay datagrams are too small" });
    expect(screen.getByText(c.noKeyframesOnLink)).toBeTruthy();
    expect(screen.getByText(c.degraded["wfb-relay"])).toBeTruthy();
  });

  it("falls back to the agent's reason for a link it has no words for", () => {
    show({ bearer: "mesh", keyframesCarried: false, degradedReason: "mesh too slow" });
    expect(screen.getByText(c.noKeyframesOnLink)).toBeTruthy();
    expect(screen.getByText("mesh too slow")).toBeTruthy();
  });

  it("shows no degraded row when the link carries keyframes", () => {
    show({ bearer: "direct-lan", keyframesCarried: true });
    expect(screen.queryByText(c.noKeyframesOnLink)).toBeNull();
  });

  it("shows waiting-for-anchor, the cap, drops and the pose source", () => {
    show({ anchored: false, capped: true, droppedKeyframes: 3, poseTier: "vio" });
    expect(screen.getByText(c.waitingForAnchor)).toBeTruthy();
    expect(screen.getByText(c.cappedNotice)).toBeTruthy();
    expect(screen.getByText(c.droppedKeyframes)).toBeTruthy();
    expect(screen.getByText("3")).toBeTruthy();
    expect(screen.getByText(c.poseTierLabel)).toBeTruthy();
    expect(screen.getByText(c.poseTier.vio)).toBeTruthy();
  });

  it("shows no anchor or cap notice for an anchored, uncapped capture", () => {
    show({ anchored: true, capped: false });
    expect(screen.queryByText(c.waitingForAnchor)).toBeNull();
    expect(screen.queryByText(c.cappedNotice)).toBeNull();
  });

  it("has no relay decimation row (the agent does not publish it)", () => {
    show({ bearer: "wfb-relay" });
    expect(screen.queryByText("Relay decimation")).toBeNull();
    expect("relayDecimation" in EMPTY_ATLAS_LIVE).toBe(false);
    expect("relayGroundAgentId" in EMPTY_ATLAS_LIVE).toBe(false);
  });

  it("words tracking health and bearer from the catalogue, with an Other fallback", () => {
    show({ vioHealth: "degraded", bearer: "carrier-pigeon" });
    expect(screen.getByText(c.health.degraded)).toBeTruthy();
    expect(screen.getByText(c.vioTracking)).toBeTruthy();
    expect(screen.getByText(c.bearerOther)).toBeTruthy();
    expect(screen.queryByText("carrier-pigeon")).toBeNull();
    cleanup();
    show({ vioHealth: "wobbly" });
    expect(screen.queryByText("wobbly")).toBeNull();
    expect(screen.getByText(c.health.other)).toBeTruthy();
  });

  it("names the compute node and keeps its id in the title", () => {
    show({ computeNodeId: "ws-7f3a9c" }, [{ deviceId: "ws-7f3a9c", name: "Bench workstation" }]);
    const cell = screen.getByText("Bench workstation");
    expect(cell.getAttribute("title")).toBe("ws-7f3a9c");
    cleanup();
    show({ computeNodeId: "ws-unknown" });
    expect(screen.getByText("ws-unknown")).toBeTruthy();
  });
});
