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

/** Steps the operator fills in from a measurement or the CAD model and that
 * write straight into `rig.json` (no solver runs): the LiDAR mounting
 * extrinsic and the GNSS antenna lever arm. */
export type EntryTarget = "lidar_imu" | "gnss_lever_arm";

export type WizardStep = { kind: "job"; step: CalibStep } | { kind: "entry"; target: EntryTarget };

/** The whole wizard in order (IMU noise → intrinsics → camera–IMU → LiDAR–IMU
 * → LiDAR–camera → GNSS lever arm), keeping only what the rig carries. */
export function wizardFor(rig: { hasLidar: boolean; hasGnss: boolean }): WizardStep[] {
  const job = (target: CalibTarget): WizardStep[] => {
    const step = CALIB_STEPS.find((s) => s.target === target);
    return step && (rig.hasLidar || !step.needsLidar) ? [{ kind: "job", step }] : [];
  };
  return [
    ...job("imu_noise"),
    ...job("intrinsics"),
    ...job("camera_imu"),
    ...(rig.hasLidar ? [{ kind: "entry" as const, target: "lidar_imu" as const }] : []),
    ...job("lidar_camera"),
    ...(rig.hasGnss ? [{ kind: "entry" as const, target: "gnss_lever_arm" as const }] : []),
  ];
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
