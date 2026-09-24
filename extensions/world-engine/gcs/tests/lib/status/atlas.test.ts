/**
 * `mapAtlasSlice`: maps the `atlas` slice of the extension's agent state (the
 * capture service's own report) into the atlas store's live slice. Covers the
 * empty-slice no-op, full + sparse mapping (merge over current), and defensive
 * coercion.
 */

import { describe, it, expect } from "vitest";
import { mapAtlasSlice } from "../../../src/lib/status/atlas";
import { EMPTY_ATLAS_LIVE } from "../../../src/stores/atlas-store";

const current = { live: { ...EMPTY_ATLAS_LIVE } };

describe("mapAtlasSlice", () => {
  it("returns null when the slice carries no atlas field", () => {
    // A present-but-empty slice carries nothing to merge.
    expect(mapAtlasSlice({}, current, 1)).toBeNull();
    // Unknown keys and wrong-typed known keys are not atlas facts either.
    expect(mapAtlasSlice({ other: { x: 1 } }, current, 1)).toBeNull();
    expect(
      mapAtlasSlice({ state: "", keyframesIngested: "12", capped: "yes" }, current, 1),
    ).toBeNull();
  });

  it("maps the atlas slice's capture + transport fields", () => {
    const patch = mapAtlasSlice(
      {
        state: "capturing",
        sessionId: "sess-1",
        keyframesIngested: 142,
        ingestRateHz: 9.5,
        cameraCount: 6,
        vioHealth: "good",
        computeNodeId: "rtx-box",
        lastKfAt: 1700,
        bearer: "direct-lan",
      },
      current,
      999,
    );
    expect(patch?.live).toMatchObject({
      state: "capturing",
      sessionId: "sess-1",
      keyframesIngested: 142,
      ingestRateHz: 9.5,
      cameraCount: 6,
      vioHealth: "good",
      computeNodeId: "rtx-box",
      lastKfAt: 1700,
      bearer: "direct-lan",
      updatedAt: 999,
    });
  });

  it("maps the relay transport fields", () => {
    const patch = mapAtlasSlice(
      {
        state: "capturing",
        bearer: "wfb-relay",
        relayGroundAgentId: "gs-01",
        relayDecimation: 4,
      },
      current,
      5,
    );
    expect(patch?.live.bearer).toBe("wfb-relay");
    expect(patch?.live.relayGroundAgentId).toBe("gs-01");
    expect(patch?.live.relayDecimation).toBe(4);
  });

  // Each capture-honesty field exists because its absence made a known-false
  // reading indistinguishable from a healthy one.
  it("maps the capture-honesty fields (capped / anchored / pose tier / drops)", () => {
    const patch = mapAtlasSlice(
      {
        state: "capturing",
        keyframesIngested: 400,
        capped: true,
        anchored: false,
        poseTier: "offloaded_slam",
        droppedKeyframes: 7,
      },
      current,
      11,
    );
    // `capped` is why a frozen keyframe count against "capturing" is not a
    // stalled camera; `anchored: false` is a capture that is running and
    // producing nothing; `droppedKeyframes` is why the ingested count overstates
    // the reconstruction input.
    expect(patch?.live.capped).toBe(true);
    expect(patch?.live.anchored).toBe(false);
    expect(patch?.live.poseTier).toBe("offloaded_slam");
    expect(patch?.live.droppedKeyframes).toBe(7);
  });

  it("maps keyframesCarried, including the false the WFB relay reports", () => {
    // A `false` here means the world-model lane is degraded to pose and status —
    // no world model — while the bearer still reads as live. It must survive the
    // merge rather than being treated as absent.
    const patch = mapAtlasSlice(
      { state: "capturing", bearer: "wfb-relay", keyframesCarried: false },
      current,
      13,
    );
    expect(patch?.live.keyframesCarried).toBe(false);
  });

  it("a false honesty flag alone is still a fact worth merging", () => {
    const prior = { live: { ...EMPTY_ATLAS_LIVE, anchored: true } };
    expect(mapAtlasSlice({ anchored: false }, prior, 2)?.live.anchored).toBe(false);
  });

  it("leaves the honesty fields null when the agent reports none", () => {
    // No claim either way is the honest reading for a pre-field agent, so these
    // must not default to a confident false.
    const patch = mapAtlasSlice({ state: "idle" }, current, 17);
    expect(patch?.live.keyframesCarried).toBeNull();
    expect(patch?.live.capped).toBeNull();
    expect(patch?.live.anchored).toBeNull();
    expect(patch?.live.poseTier).toBeNull();
    expect(patch?.live.droppedKeyframes).toBeNull();
  });

  it("merges a sparse slice over the current slice", () => {
    const prior = {
      live: {
        ...EMPTY_ATLAS_LIVE,
        state: "capturing",
        sessionId: "sess-1",
        keyframesIngested: 100,
        computeNodeId: "rtx-box",
      },
    };
    // Only the keyframe count changes; everything else is preserved.
    const patch = mapAtlasSlice({ keyframesIngested: 110 }, prior, 7);
    expect(patch?.live).toMatchObject({
      state: "capturing",
      sessionId: "sess-1",
      keyframesIngested: 110,
      computeNodeId: "rtx-box",
      updatedAt: 7,
    });
  });

  it("ignores non-finite numerics (treated as absent)", () => {
    const patch = mapAtlasSlice(
      { state: "capturing", ingestRateHz: Number.NaN, cameraCount: Infinity },
      current,
      1,
    );
    expect(patch?.live.state).toBe("capturing");
    expect(patch?.live.ingestRateHz).toBeNull();
    expect(patch?.live.cameraCount).toBeNull();
  });
});
