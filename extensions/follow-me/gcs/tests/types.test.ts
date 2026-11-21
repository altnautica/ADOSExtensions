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
      // A payload with no reacquire_mode reads as strict, never smart; with no
      // uncertain_mode as hold; with no predictive_search as off.
      reacquireMode: "strict",
      uncertainMode: "hold",
      predictiveSearch: false,
    });
  });

  it("maps phase, range cues, AGL, re-acquire, latency and setpoint rate", () => {
    const s = normalizeFollowState({
      phase: "searching",
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
      reacquire_remaining_s: 18.4,
      pipeline_latency_ms: 85,
      setpoint_rate_hz: 20,
      confidence: 0.97,
    });
    expect(s.phase).toBe("searching");
    expect(s.rangeSigmaM).toBe(0.3);
    expect(s.rangeCues).toEqual([{ kind: "ground", rangeM: 10.1, sigmaM: 0.4 }]);
    expect(s.aglSource).toBe("rng");
    expect(s.reacquireMode).toBe("smart");
    expect(s.reacquireRemainingS).toBe(18.4);
    expect(s.pipelineLatencyMs).toBe(85);
    expect(s.setpointRateHz).toBe(20);
    expect(s.confidence).toBe(0.97);
  });

  it("reads a raised follow height, and none when the payload says nothing", () => {
    expect(normalizeFollowState({ height_setpoint_m: 3.4, height_raised: true }).heightRaised).toBe(true);
    expect(normalizeFollowState({ height_setpoint_m: 4 }).heightRaised).toBe(false);
  });

  it("reads why the height flown is lowered under trees, and nothing else", () => {
    expect(normalizeFollowState({ height_setpoint_m: 3.8, height_reason: "canopy" }).heightReason).toBe("canopy");
    expect(normalizeFollowState({ height_reason: "canopy_floor" }).heightReason).toBe("canopy_floor");
    expect(normalizeFollowState({ height_reason: "weather" }).heightReason).toBeNull();
    expect(normalizeFollowState({}).heightReason).toBeNull();
  });

  it("reads the subject source the follow flies on, and nothing else", () => {
    expect(normalizeFollowState({ subject_source: "beacon" }).subjectSource).toBe("beacon");
    expect(normalizeFollowState({ subject_source: "predicted" }).subjectSource).toBe("predicted");
    expect(normalizeFollowState({ subject_source: "vision" }).subjectSource).toBe("vision");
    expect(normalizeFollowState({ subject_source: "radar" }).subjectSource).toBeNull();
    expect(normalizeFollowState({}).subjectSource).toBeNull();
  });

  it("reads the range's shortest standoff, whether it is held there and why", () => {
    const s = normalizeFollowState({ standoff_min_m: 5, standoff_raised: true, standoff_reason: "subject-standing" });
    expect([s.standoffMinM, s.standoffRaised, s.standoffReason]).toEqual([5, true, "subject-standing"]);
    const odd = normalizeFollowState({ standoff_min_m: 5, standoff_raised: true, standoff_reason: "weather" });
    expect(odd.standoffReason).toBeNull();
    const held = normalizeFollowState({ standoff_raised: false, standoff_reason: "camera-range" });
    expect(held.standoffReason).toBeNull();
    const none = normalizeFollowState({});
    expect([none.standoffMinM, none.standoffRaised, none.standoffReason]).toEqual([null, false, null]);
  });

  it("normalizes the 0.5 recovery fields and phases", () => {
    const t = createPluginHarness({
      mount: () => undefined,
      locale: enLocale as Record<string, string>,
    }).ctx.i18n.t;
    const s = normalizeFollowState({
      active: true,
      phase: "holding-await-operator",
      hold_reason: "search-exhausted",
      hold_ne: [1, 2],
      reacquire_remaining_s: 0,
      predictive_search: true,
      uncertain_mode: "follow",
    });
    expect(s.phase).toBe("holding-await-operator");
    expect(s.holdNe).toEqual([1, 2]);
    expect(t(holdReasonLabelKey(s.holdReason!))).toBe("Search found no match");
    expect(s.predictiveSearch).toBe(true);
    expect(s.uncertainMode).toBe("follow");
  });

  it("maps the search, vantage and occluder fields", () => {
    const s = normalizeFollowState({
      phase: "searching",
      search_budget_s: 25,
      search_step: 1,
      search_area: { center_ne: [12.1, -3], semi_major_m: 3.2, semi_minor_m: 1.4, heading_deg: 41 },
      predicted_path_ne: [[10, -2], [10.6, -2.4], [1, "x"]],
      vantage_ne: [8, 2.5],
      vantage_state: "enroute",
      occluder: { ne: [11, -1], kind: "narrow" },
      operator_resume_seq: 3,
    });
    expect(s.searchBudgetS).toBe(25);
    expect(s.searchStep).toBe(1);
    expect(s.searchArea).toEqual({ centerNe: [12.1, -3], semiMajorM: 3.2, semiMinorM: 1.4, headingDeg: 41 });
    expect(s.predictedPathNe).toEqual([[10, -2], [10.6, -2.4]]);
    expect(s.vantageNe).toEqual([8, 2.5]);
    expect(s.vantageState).toBe("enroute");
    expect(s.occluder).toEqual({ ne: [11, -1], kind: "narrow" });
    expect(s.operatorResumeSeq).toBe(3);
  });

  it("drops unknown recovery words and reads unknown modes conservatively", () => {
    const s = normalizeFollowState({
      vantage_state: "bogus",
      occluder: { ne: [1, 2], kind: "bogus" },
      uncertain_mode: "bogus",
      predictive_search: "yes" as never,
      hold_ne: [1],
    });
    expect(s.vantageState).toBeNull();
    expect(s.occluder).toBeNull();
    expect(s.uncertainMode).toBe("hold");
    expect(s.predictiveSearch).toBe(false);
    expect(s.holdNe).toBeNull();
  });

  it("no longer knows the retired coasting and lost phases", () => {
    expect(normalizeFollowState({ phase: "coasting" }).phase).toBe("idle");
    expect(normalizeFollowState({ phase: "lost" }).phase).toBe("idle");
  });

  it("reads the outpaced flag and the speed limit", () => {
    const s = normalizeFollowState({ outpaced: true, speed_limit_mps: 2 });
    expect(s.outpaced).toBe(true);
    expect(s.speedLimitMps).toBe(2);
    const n = normalizeFollowState({ outpaced: "yes" as never });
    expect(n.outpaced).toBe(false);
    expect(n.speedLimitMps).toBeNull();
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

  it("carries the camera-lost hold reason", () => {
    expect(normalizeFollowState({ hold_reason: "camera-lost" }).holdReason).toBe(
      "camera-lost",
    );
  });

  it("carries the camera geometry in use", () => {
    const s = normalizeFollowState({
      camera_hfov_deg: 87,
      camera_mount_pitch_deg: -15,
      camera_geometry_source: "roster",
    });
    expect(s.cameraHfovDeg).toBe(87);
    expect(s.cameraMountPitchDeg).toBe(-15);
    expect(s.cameraGeometrySource).toBe("roster");
    const none = normalizeFollowState({ camera_geometry_source: "bogus" });
    expect(none.cameraHfovDeg).toBeNull();
    expect(none.cameraGeometrySource).toBeNull();
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
    "awaiting-operator",
    "search-exhausted",
    "predictive-search-off",
    "pose-stale",
    "fc-stale",
    "fc-disarmed",
    "no-range",
    "lease-busy",
    "lease-refused",
    "camera-lost",
  ] as const;

  it("drops the retired lock-lost reason", () => {
    expect(normalizeFollowState({ hold_reason: "lock-lost" }).holdReason).toBeNull();
  });

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
      "phase.holding",
      "phase.predicting",
      "phase.repositioning",
      "phase.searching",
      "phase.holding-await-operator",
      "reacquire.strict",
      "reacquire.smart",
      "uncertain_mode.follow",
      "uncertain_mode.hold",
      "vantage.enroute",
      "vantage.arrived",
      "vantage.no-vantage",
      "occluder.narrow",
      "occluder.wide",
      "prompt.title",
      "prompt.body.awaiting-operator",
      "prompt.body.search-exhausted",
      "prompt.body.predictive-search-off",
      "prompt.body.no-range",
      "cue.ground",
      "cue.box_height",
      "cue.lidar",
      "agl.rng",
      "agl.baro",
    ];
    for (const key of keys) {
      expect((enLocale as Record<string, string>)[key], key).toBeTruthy();
    }
    for (const retired of ["phase.coasting", "phase.lost", "metrics.coastRemaining", "hold.lockLost"]) {
      expect((enLocale as Record<string, string>)[retired], retired).toBeUndefined();
    }
  });

  it("words the wait for the operator without assuming the mode", () => {
    // The same hold reason follows a Smart loss before any range fix.
    const body = (enLocale as Record<string, string>)["prompt.body.awaiting-operator"];
    expect(body).not.toMatch(/strict|smart/i);
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
    expect(c.reacquire_mode).toBe("smart");
    expect(c.max_speed_mps).toBe(6);
  });

  it("reads an unknown re-acquire mode as strict", () => {
    expect(normalizeConfig({ reacquire_mode: "x" }).reacquire_mode).toBe("strict");
    expect(normalizeConfig({ reacquire_mode: "smart" }).reacquire_mode).toBe("smart");
  });

  it("coerces a numeric string", () => {
    expect(normalizeConfig({ follow_distance_m: "12" }).follow_distance_m).toBe(12);
  });

  it("reads the recovery settings and has no re-acquire window", () => {
    const d = normalizeConfig({});
    expect(d.uncertain_mode).toBe("follow");
    expect(d.predictive_search).toBe(true);
    expect([d.search_budget_s, d.search_radius_m, d.search_speed_mps]).toEqual([25, 12, 2]);
    expect("reacquire_window_s" in d).toBe(false);
    const c = normalizeConfig({
      uncertain_mode: "bogus",
      predictive_search: false,
      search_budget_s: "40",
    });
    expect(c.uncertain_mode).toBe("hold");
    expect(c.predictive_search).toBe(false);
    expect(c.search_budget_s).toBe(40);
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
