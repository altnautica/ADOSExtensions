/**
 * `applyCaptureState`: the link's `{"capture": …}` report fans into the
 * reporting device's live state, merged over what that device said before.
 */

import { beforeEach, describe, expect, it } from "vitest";

import { applyCaptureState } from "../../src/hooks/use-capture-state";
import { EMPTY_CAPTURE_LIVE, useCaptureStore } from "../../src/stores/capture-store";

const live = (deviceId: string) => useCaptureStore.getState().live[deviceId] ?? EMPTY_CAPTURE_LIVE;

beforeEach(() => {
  useCaptureStore.setState({ live: {} });
});

describe("applyCaptureState", () => {
  it("fans the capture slice into the reporting device's live state", () => {
    applyCaptureState(
      "drone-1",
      { capture: { state: "capturing", keyframesIngested: 12, bearer: "direct-lan" } },
      500,
    );
    expect(live("drone-1")).toMatchObject({
      state: "capturing",
      keyframesIngested: 12,
      bearer: "direct-lan",
      updatedAt: 500,
    });
    expect(useCaptureStore.getState().live["drone-2"]).toBeUndefined();
  });

  it("merges a sparse report over the device's previous one", () => {
    applyCaptureState("drone-1", { capture: { state: "capturing", sessionId: "s1" } }, 1);
    applyCaptureState("drone-1", { capture: { keyframesIngested: 40 } }, 2);
    expect(live("drone-1")).toMatchObject({
      state: "capturing",
      sessionId: "s1",
      keyframesIngested: 40,
      updatedAt: 2,
    });
  });

  it("ignores a report that is not an object, a null slice and an empty one", () => {
    for (const payload of [null, "status", 3, { capture: null }, { capture: ["capturing"] }]) {
      applyCaptureState("drone-1", payload, 1);
    }
    expect(useCaptureStore.getState().live).toEqual({});
    applyCaptureState("drone-1", { capture: { state: "capturing" } }, 1);
    const before = useCaptureStore.getState().live;
    applyCaptureState("drone-1", { capture: {} }, 2);
    expect(useCaptureStore.getState().live).toBe(before);
  });
});
