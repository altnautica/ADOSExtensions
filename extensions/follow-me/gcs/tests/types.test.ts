import { describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import {
  EMPTY_FOLLOW_STATE,
  holdReasonLabelKey,
  isStaleHold,
  modeSummary,
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
      ...EMPTY_FOLLOW_STATE,
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
      // A payload with no reacquire_mode reads as strict, never smart.
      reacquireMode: "strict",
    });
  });

  it("maps phase, range cues, AGL, re-acquire, latency and setpoint rate", () => {
    const s = normalizeFollowState({
      phase: "coasting",
      range_m: 10.2,
      range_sigma_m: 0.3,
      range_cues: [
        { kind: "ground", range_m: 10.1, sigma_m: 0.4 },
        { kind: "bogus", range_m: 1, sigma_m: 1 },
      ],
      target_speed_mps: 1.4,
      agl_m: 4.1,
      agl_source: "rng",
      reacquire_mode: "smart",
      coast_remaining_s: 2.5,
      pipeline_latency_ms: 85,
      setpoint_rate_hz: 20,
      confidence: 0.97,
    });
    expect(s.phase).toBe("coasting");
    expect(s.rangeSigmaM).toBe(0.3);
    expect(s.rangeCues).toEqual([{ kind: "ground", rangeM: 10.1, sigmaM: 0.4 }]);
    expect(s.aglSource).toBe("rng");
    expect(s.reacquireMode).toBe("smart");
    expect(s.coastRemainingS).toBe(2.5);
    expect(s.pipelineLatencyMs).toBe(85);
    expect(s.setpointRateHz).toBe(20);
    expect(s.confidence).toBe(0.97);
  });

  it("reads an unknown phase as idle and an unknown mode as strict", () => {
    const s = normalizeFollowState({ phase: "bogus", reacquire_mode: "bogus" });
    expect(s.phase).toBe("idle");
    expect(s.reacquireMode).toBe("strict");
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
    "no-range",
    "lease-busy",
    "lease-refused",
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

  it("has a locale label for every phase, mode, cue and AGL source", () => {
    const keys = [
      "phase.idle",
      "phase.acquiring",
      "phase.following",
      "phase.coasting",
      "phase.searching",
      "phase.lost",
      "phase.holding",
      "reacquire.strict",
      "reacquire.smart",
      "cue.ground",
      "cue.box_height",
      "cue.lidar",
      "agl.rng",
      "agl.baro",
    ];
    for (const key of keys) {
      expect((enLocale as Record<string, string>)[key], key).toBeTruthy();
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
    expect(c.designate_camera).toBe("auto");
    expect(c.camera_hfov_deg).toBe(70);
    expect(c.mount_pitch_deg).toBe(30);
    expect(c.reacquire_mode).toBe("smart");
    expect(c.max_speed_mps).toBe(6);
  });

  it("reads an unknown re-acquire mode as strict", () => {
    expect(normalizeConfig({ reacquire_mode: "x" }).reacquire_mode).toBe("strict");
    expect(normalizeConfig({ reacquire_mode: "smart" }).reacquire_mode).toBe("smart");
  });

  it("coerces a numeric string", () => {
    expect(normalizeConfig({ camera_hfov_deg: "90" }).camera_hfov_deg).toBe(90);
  });
});

describe("follow modes", () => {
  const { ctx } = createPluginHarness({
    mount: () => undefined,
    locale: enLocale as Record<string, string>,
  });
  const t = ctx.i18n.t;

  it("normalizes the mode, orbit and lease fields", () => {
    const s = normalizeFollowState({
      follow_mode: "orbit",
      orbit_radius_m: 12,
      orbit_rate_dps: 15,
      orbit_direction: "ccw",
      orbit_phase_deg: 90,
      lease_id: 7,
      nudged: true,
    });
    expect(s.followMode).toBe("orbit");
    expect(s.orbitDirection).toBe("ccw");
    expect(s.orbitPhaseDeg).toBe(90);
    expect(s.leaseId).toBe(7);
    expect(s.nudged).toBe(true);
    expect(modeSummary(s, null, t)).toBe("Orbit 12 m · 15°/s · Counter-clockwise");
  });

  it("defaults unknown words to follow and clockwise", () => {
    const s = normalizeFollowState({ follow_mode: "x", orbit_direction: "y" });
    expect(s.followMode).toBe("follow");
    expect(s.orbitDirection).toBe("cw");
    expect(s.leaseId).toBeNull();
    expect(modeSummary({ ...s, followAngleDeg: 90 }, 8, t)).toBe("Follow 90° · 8 m");
  });

  it("drops the retired fc-not-guided reason", () => {
    expect(normalizeFollowState({ hold_reason: "fc-not-guided" }).holdReason).toBeNull();
  });
});
