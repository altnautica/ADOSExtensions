/**
 * @module capture/calibration
 * @description The calibration wizard's steps, the device's stored calibration
 * versions (`GET atlas/calib`) and a node `calibrate` job's report.
 */

import { arr, num, obj, objOrNull, str } from "./coerce";

/** Node `calibrate` job targets, in the order the wizard runs them. */
export const CALIB_TARGETS = ["imu_noise", "intrinsics", "camera_imu", "lidar_camera"] as const;
export type CalibTarget = (typeof CALIB_TARGETS)[number];

export interface CalibStep {
  target: CalibTarget;
  /** Recording the operator makes for this step, in seconds (guidance). */
  recordS: number;
  /** Only offered when the rig carries a LiDAR. */
  needsLidar: boolean;
}

export const CALIB_STEPS: readonly CalibStep[] = [
  { target: "imu_noise", recordS: 7200, needsLidar: false },
  { target: "intrinsics", recordS: 90, needsLidar: false },
  { target: "camera_imu", recordS: 90, needsLidar: false },
  { target: "lidar_camera", recordS: 30, needsLidar: true },
];

/** The steps a rig needs: the LiDAR step only when the rig has a LiDAR. */
export function stepsFor(hasLidar: boolean): CalibStep[] {
  return CALIB_STEPS.filter((s) => hasLidar || !s.needsLidar);
}

export interface CalibVersion {
  calibVersion: string;
  createdUtc: string | null;
  source: string | null;
  reprojectionRmsPx: number | null;
}

export interface CalibStore {
  rigId: string | null;
  active: string | null;
  versions: CalibVersion[];
}

/** Coerce a `GET atlas/calib` body. */
export function coerceCalibStore(raw: unknown): CalibStore {
  const e = obj(raw);
  return {
    rigId: str(e.rig_id),
    active: str(e.active),
    versions: arr(e.versions).flatMap((v) => {
      const r = obj(v);
      const calibVersion = str(r.calib_version);
      if (!calibVersion) return [];
      return [
        {
          calibVersion,
          createdUtc: str(r.created_utc),
          source: str(r.source),
          reprojectionRmsPx: num(obj(r.report).reprojection_rms_px),
        },
      ];
    }),
  };
}

export interface CalibReport {
  target: string | null;
  reprojectionRmsPx: number | null;
  tool: string | null;
  toolVersion: string | null;
  /** Every other numeric field, for a key/value readout. */
  residuals: { key: string; value: number }[];
}

const REPORT_KEYS: Record<string, true> = { target: true, reprojection_rms_px: true, tool: true, tool_version: true };

/** Coerce a node `calib_report` output, or null when it is not an object. */
export function coerceCalibReport(raw: unknown): CalibReport | null {
  const e = objOrNull(raw);
  if (!e) return null;
  return {
    target: str(e.target),
    reprojectionRmsPx: num(e.reprojection_rms_px),
    tool: str(e.tool),
    toolVersion: str(e.tool_version),
    residuals: Object.entries(e).flatMap(([key, value]) =>
      !REPORT_KEYS[key] && typeof value === "number" && Number.isFinite(value) ? [{ key, value }] : [],
    ),
  };
}
