/**
 * Splat choice (LOD before SPZ before PLY) and its placement sidecar, the
 * output frame rule, the legacy COLMAP frame rule,
 * warehouse aisles, coverage bounds at warehouse scale, the capture poll
 * memory, walk key gating, and the measured rig-entry math.
 */

import { describe, expect, it } from "vitest";
import * as THREE from "three";

import { pickArtifactForViewer, pickSplatOutput, splatPlaceJson } from "../../../src/components/atlas/viewer-types";
import { frameWithoutSidecar } from "../../../src/hooks/use-output-frame";
import type { ComputeOutput } from "../../../src/lib/agent/compute-client";
import { coverageBounds } from "../../../src/components/capture/CoverageMap";
import { walkKeyApplies } from "../../../src/components/world/walk-controls";
import { applyPoll, STALE_AFTER_MISSES } from "../../../src/hooks/use-capture";
import { rigCalibShape, wizardFor, type WizardStep } from "../../../src/lib/capture/calibration";
import {
  manualCalibVersion,
  transformFromXyzRpy,
  withExtrinsic,
  withLeverArm,
  xyzRpyFromTransform,
} from "../../../src/lib/capture/rig-entry";
import type { LidarModel, Rig } from "../../../src/lib/contract/generated";
import { translator } from "../../../src/host/i18n";
import messages from "../../../../locales/en.json";
import { aisleLinesViewer, coerceAisles } from "../../../src/lib/world/aisles";
import { artifactToViewerMatrix } from "../../../src/lib/world/placement";

const t = translator("calibration");

const MID360: LidarModel = {
  model: "livox-mid360",
  pattern: "solid",
  h_fov_deg: 360,
  v_min_deg: -7,
  v_max_deg: 52,
  rate_hz: 10,
  points_per_s: 200000,
  min_range_m: 0.1,
  max_range_m: 40,
  range_sigma_m: 0.02,
};

const C1: LidarModel = {
  model: "rplidar-c1",
  pattern: "planar",
  h_fov_deg: 360,
  v_min_deg: 0,
  v_max_deg: 0,
  rate_hz: 10,
  points_per_s: 5000,
  min_range_m: 0.05,
  max_range_m: 12,
  range_sigma_m: 0.03,
  blocked_azimuth_deg: [[130, 230]],
};

describe("pickSplatOutput", () => {
  const outs = [
    { kind: "splat", format: "ply", file: "scene.ply", placeJson: "scene.ply.place.json" },
    { kind: "splat_lod", format: "rad", file: "scene-lod-0.rad", placeJson: null },
    { kind: "splat", format: "spz", file: "scene.spz", placeJson: "scene.spz.place.json" },
    { kind: "pointcloud", format: "laz", file: "cloud.laz", placeJson: "cloud.laz.place.json" },
  ];

  it("prefers the streamed RAD LOD, then SPZ, then the raw PLY, whatever the listing order", () => {
    expect(pickSplatOutput(outs)?.file).toBe("scene-lod-0.rad");
    expect(pickSplatOutput(outs.filter((o) => o.format !== "rad"))?.file).toBe("scene.spz");
    expect(pickArtifactForViewer([outs[0]!, outs[3]!], "splat")?.file).toBe("scene.ply");
    // No format field: the file extension decides.
    expect(pickSplatOutput([{ kind: "splat", file: "a.ply" }, { kind: "splat", file: "b.spz" }])?.file).toBe("b.spz");
  });

  it("gives a .rad the SPZ's placement sidecar (it has none of its own)", () => {
    expect(splatPlaceJson(outs, outs[1])).toBe("scene.spz.place.json");
    expect(splatPlaceJson(outs, outs[0])).toBe("scene.ply.place.json");
    // A non-splat never borrows a splat's sidecar.
    expect(splatPlaceJson([outs[1]!, { kind: "mesh", format: "glb", file: "m.glb", placeJson: null }], outs[1])).toBeNull();
    expect(splatPlaceJson(outs, undefined)).toBeNull();
  });
});

describe("frameWithoutSidecar", () => {
  const out = (kind: string): ComputeOutput => ({
    id: "o",
    jobId: "j",
    kind,
    uri: "",
    source: null,
    backend: null,
    meta: null,
    file: null,
    format: null,
    sizeBytes: null,
    placeJson: null,
    createdMs: 0,
  });

  it("reads a named sidecar, treats a sidecar-less LOD as local ENU and anything else as legacy", () => {
    expect(frameWithoutSidecar(out("splat_lod"), "scene.spz.place.json")).toBeUndefined();
    expect(frameWithoutSidecar(out("splat_lod"), null)).toBeNull();
    expect(frameWithoutSidecar(out("splat"), null)).toBe("colmap");
  });
});

describe("legacy COLMAP frame", () => {
  it("is the same 180°-about-X turn the legacy cloud viewer applies", () => {
    const m = new THREE.Matrix4().set(...(artifactToViewerMatrix("colmap") as Parameters<THREE.Matrix4["set"]>));
    expect(new THREE.Vector3(2, 3, 5).applyMatrix4(m).toArray()).toEqual([2, -3, -5]);
  });
});

describe("aisles", () => {
  it("reads LineString aisles and draws centreline plus both edges", () => {
    const aisles = coerceAisles({
      type: "FeatureCollection",
      frame: "enu_local",
      features: [
        { geometry: { type: "LineString", coordinates: [[0, 0], [10, 0]] }, properties: { id: "A1", width_m: 2, length_m: 10 } },
        { geometry: { type: "Polygon", coordinates: [] } },
        { geometry: { type: "LineString", coordinates: [[0, 0]] } },
      ],
    });
    expect(aisles).toEqual([{ id: "A1", line: [[0, 0], [10, 0]], widthM: 2, lengthM: 10 }]);
    const lines = Array.from(aisleLinesViewer(aisles, 0.1));
    expect(lines).toHaveLength(18); // centreline + 2 edges, 2 vertices × 3 each
    // Left edge at north +1 m → viewer z −1.
    expect(lines.slice(6, 12).map((v) => Math.round(v * 100) / 100)).toEqual([0, 0.1, -1, 10, 0.1, -1]);
  });
});

describe("coverageBounds", () => {
  it("handles a 160k-cell warehouse grid without spreading arguments", () => {
    const cells = [];
    for (let ix = 0; ix < 400; ix++) for (let iy = 0; iy < 400; iy++) cells.push({ ix, iy, hits: 1 });
    const b = coverageBounds({ cellM: 0.25, cells, trajectory: [[-1, 2, 0]], start: null, current: null, headingDeg: null });
    expect(b).toEqual({ minX: -1, minY: 0, maxX: 100, maxY: 100 });
    expect(coverageBounds({ cellM: 0.25, cells: [], trajectory: [], start: null, current: null, headingDeg: null })).toBeNull();
  });
});

describe("applyPoll", () => {
  it("keeps the last good value through misses and counts them", () => {
    let m = applyPoll({ value: null as number | null, misses: 0 }, 1);
    for (let i = 0; i < STALE_AFTER_MISSES; i++) m = applyPoll(m, null);
    expect(m).toEqual({ value: 1, misses: STALE_AFTER_MISSES });
    expect(applyPoll(m, 2)).toEqual({ value: 2, misses: 0 });
  });
});

describe("walkKeyApplies", () => {
  it("moves only when the viewer itself has the key event", () => {
    const canvas = document.createElement("canvas");
    expect(walkKeyApplies(canvas, canvas)).toBe(true);
    for (const tag of ["input", "textarea", "select", "button"]) {
      expect(walkKeyApplies(document.createElement(tag), canvas)).toBe(false);
    }
    expect(walkKeyApplies(document.body, canvas)).toBe(false);
  });
});

describe("rig entry", () => {
  const rig: Rig = {
    contract: "ados.capture/1",
    rig_id: "pro-01",
    calib_version: "v1",
    sensors: [
      {
        id: "lidar0",
        kind: "lidar",
        role: "up",
        T_body_sensor: transformFromXyzRpy([0, 0, 0], [0, 0, 0]),
        time_offset_ns: 0,
        lidar: MID360,
      },
      { id: "gnss0", kind: "gnss", role: "nav", T_body_sensor: transformFromXyzRpy([0, 0, 0], [0, 0, 0]), time_offset_ns: 0 },
    ],
  };

  it("round-trips translation and roll/pitch/yaw", () => {
    const back = xyzRpyFromTransform(transformFromXyzRpy([0.1, -0.02, 0.15], [5, -10, 30]));
    back.xyz.forEach((v, i) => expect(v).toBeCloseTo([0.1, -0.02, 0.15][i]!, 12));
    back.rpy.forEach((v, i) => expect(v).toBeCloseTo([5, -10, 30][i]!, 9));
  });

  it("writes a new version without touching the source rig", () => {
    const t = transformFromXyzRpy([0, 0, 0.12], [0, 0, 180]);
    const a = withExtrinsic(rig, "lidar0", t, "v2");
    expect(a.calib_version).toBe("v2");
    expect(a.sensors[0]?.T_body_sensor).toBe(t);
    expect(rig.sensors[0]?.T_body_sensor).not.toBe(t);
    const b = withLeverArm(rig, "gnss0", [0, 0, 0.112], "v3");
    expect(b.sensors[1]?.lever_arm_m).toEqual([0, 0, 0.112]);
    expect(rig.sensors[1]?.lever_arm_m).toBeUndefined();
    expect(manualCalibVersion(new Date("2026-10-19T10:15:00.123Z"))).toBe("20261019T101500Z-m");
  });

  const ids = (w: WizardStep[]) => w.map((s) => (s.kind === "job" ? s.step.target : s.kind === "entry" ? s.target : s.key));

  it("orders the wizard and keeps only the rig's sensors", () => {
    expect(ids(wizardFor(rigCalibShape(rig)))).toEqual([
      "imu_noise",
      "intrinsics",
      "camera_imu",
      "lidar_imu",
      "lidar_camera",
      "gnss_lever_arm",
    ]);
    expect(ids(wizardFor(rigCalibShape(null)))).toEqual(["imu_noise", "intrinsics", "camera_imu"]);
  });

  it("replaces the LiDAR–IMU entry with a note and asks for board views on a planar rig", () => {
    const planar: Rig = {
      ...rig,
      sensors: [{ ...rig.sensors[0]!, lidar: C1 }, rig.sensors[1]!],
    };
    const w = wizardFor(rigCalibShape(planar));
    expect(ids(w)).toEqual([
      "imu_noise",
      "intrinsics",
      "camera_imu",
      "lidar.planar_no_imu",
      "lidar_camera",
      "gnss_lever_arm",
    ]);
    const lidarCamera = w.find((s) => s.kind === "job" && s.step.target === "lidar_camera");
    expect(lidarCamera?.kind === "job" && lidarCamera.hintKey).toBe("lidar_camera.planar_board");
    expect(t(w[3]?.kind === "note" ? w[3].key : "")).toBe(messages.calibration.lidar.planar_no_imu);
    expect(t("lidar_camera.planar_board")).toBe(messages.calibration.lidar_camera.planar_board);

    // A 3D rig keeps the generic hint.
    const generic = wizardFor(rigCalibShape(rig)).find((s) => s.kind === "job" && s.step.target === "lidar_camera");
    expect(generic?.kind === "job" && generic.hintKey).toBe("hint.lidar_camera");
  });
});
