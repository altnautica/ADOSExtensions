import { describe, expect, it } from "vitest";

import {
  EMPTY_FOLLOW_STATE,
  holdReasonLabelKey,
  isStaleHold,
  normalizeFollowState,
} from "../src/types";
import { normalizeConfig } from "../src/config";
import enLocale from "../../locales/en.json";

describe("normalizeFollowState", () => {
  it("maps the agent snake_case read-back onto camelCase", () => {
    const s = normalizeFollowState({
      active: true,
      lock_state: "locked",
      target_id: 12,
      range_m: 9.5,
      distance_setpoint_m: 8,
      height_setpoint_m: 4,
      commanding: true,
      fc_armed: true,
      fc_guided: true,
    });
    expect(s).toEqual({
      active: true,
      lockState: "locked",
      targetId: 12,
      rangeM: 9.5,
      distanceSetpointM: 8,
      heightSetpointM: 4,
      commanding: true,
      fcArmed: true,
      fcGuided: true,
      holdReason: null,
    });
  });

  it("falls back to the empty baseline for a null payload", () => {
    expect(normalizeFollowState(null)).toEqual(EMPTY_FOLLOW_STATE);
  });

  it("rejects an unknown lock-state string", () => {
    expect(normalizeFollowState({ lock_state: "bogus" as never }).lockState).toBeNull();
  });

  it("carries the hold reason the agent published", () => {
    // Without this the tab can only say "not commanding", which reads the
    // same for a disarmed aircraft and for one whose telemetry died.
    const s = normalizeFollowState({ commanding: false, hold_reason: "pose-stale" });
    expect(s.holdReason).toBe("pose-stale");
  });

  it("drops an unrecognized hold reason rather than rendering it raw", () => {
    expect(
      normalizeFollowState({ hold_reason: "bogus" as never }).holdReason,
    ).toBeNull();
    expect(normalizeFollowState({}).holdReason).toBeNull();
  });
});

describe("hold reasons", () => {
  const reasons = [
    "inactive",
    "no-lock",
    "lock-uncertain",
    "lock-lost",
    "pose-stale",
    "fc-stale",
    "fc-disarmed",
    "fc-not-guided",
    "no-ground-fix",
  ] as const;

  it("has a locale label for every reason the agent can publish", () => {
    // A reason with no label renders as a bare key on a status surface.
    for (const reason of reasons) {
      expect(
        (enLocale as Record<string, string>)[holdReasonLabelKey(reason)],
        `missing label for ${reason}`,
      ).toBeTruthy();
    }
  });

  it("flags only the telemetry faults as stale", () => {
    // A disarmed controller is a normal pre-flight state and must not be
    // coloured as a fault; telemetry that stopped arriving must be.
    expect(isStaleHold("pose-stale")).toBe(true);
    expect(isStaleHold("fc-stale")).toBe(true);
    expect(isStaleHold("fc-disarmed")).toBe(false);
    expect(isStaleHold("inactive")).toBe(false);
    expect(isStaleHold(null)).toBe(false);
  });
});

describe("normalizeConfig", () => {
  it("fills defaults for missing keys", () => {
    const c = normalizeConfig({ follow_distance_m: 12 });
    expect(c.follow_distance_m).toBe(12);
    expect(c.follow_height_m).toBe(4);
    expect(c.gimbal_point).toBe(true);
    expect(c.designate_camera).toBe("uvc-0");
    expect(c.camera_hfov_deg).toBe(70);
    expect(c.mount_pitch_deg).toBe(30);
  });

  it("coerces a numeric string", () => {
    expect(normalizeConfig({ camera_hfov_deg: "90" }).camera_hfov_deg).toBe(90);
  });
});
