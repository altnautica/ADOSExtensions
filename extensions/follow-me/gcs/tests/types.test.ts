import { describe, expect, it } from "vitest";

import { EMPTY_FOLLOW_STATE, normalizeFollowState } from "../src/types";
import { normalizeConfig } from "../src/config";

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
    });
    expect(s).toEqual({
      active: true,
      lockState: "locked",
      targetId: 12,
      rangeM: 9.5,
      distanceSetpointM: 8,
      heightSetpointM: 4,
      commanding: true,
    });
  });

  it("falls back to the empty baseline for a null payload", () => {
    expect(normalizeFollowState(null)).toEqual(EMPTY_FOLLOW_STATE);
  });

  it("rejects an unknown lock-state string", () => {
    expect(normalizeFollowState({ lock_state: "bogus" as never }).lockState).toBeNull();
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
  });

  it("coerces a numeric string", () => {
    expect(normalizeConfig({ camera_hfov_deg: "90" }).camera_hfov_deg).toBe(90);
  });
});
