/**
 * @module capture/rig-entry
 * @description Measured / CAD-entered rig values written straight into the
 * contract `rig.json`: a sensor's mounting extrinsic `T_body_sensor` from a
 * translation (m) and roll/pitch/yaw (degrees, body FLU, applied Z·Y·X), and
 * the GNSS antenna lever arm. Each edit produces a new calibration version;
 * the source rig is never mutated.
 */

import type { Rig } from "../contract/generated";

export type Xyz = [number, number, number];

const RAD = Math.PI / 180;

/** Row-major 4×4 `T_body_sensor` from a translation and roll/pitch/yaw
 * (degrees), rotation `Rz(yaw)·Ry(pitch)·Rx(roll)`. */
export function transformFromXyzRpy([x, y, z]: Xyz, [roll, pitch, yaw]: Xyz): number[] {
  const [cr, sr] = [Math.cos(roll * RAD), Math.sin(roll * RAD)];
  const [cp, sp] = [Math.cos(pitch * RAD), Math.sin(pitch * RAD)];
  const [cy, sy] = [Math.cos(yaw * RAD), Math.sin(yaw * RAD)];
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y,
    -sp, cp * sr, cp * cr, z,
    0, 0, 0, 1,
  ];
}

/** The translation and roll/pitch/yaw (degrees) of a row-major `T`, the
 * inverse of {@link transformFromXyzRpy} (pitch within ±90°). */
export function xyzRpyFromTransform(t: readonly number[]): { xyz: Xyz; rpy: Xyz } {
  const g = (i: number) => t[i] ?? 0;
  const pitch = Math.asin(Math.max(-1, Math.min(1, -g(8))));
  const roll = Math.atan2(g(9), g(10));
  const yaw = Math.atan2(g(4), g(0));
  return { xyz: [g(3), g(7), g(11)], rpy: [roll / RAD, pitch / RAD, yaw / RAD] };
}

/** A new calibration version id for a manual edit at `now`
 * (`20261019T101500Z-m`). */
export function manualCalibVersion(now: Date): string {
  return `${now.toISOString().replace(/[-:]/g, "").replace(/\.\d+Z$/, "Z")}-m`;
}

/** `rig` with one sensor's `T_body_sensor` replaced, under `calibVersion`. */
export function withExtrinsic(rig: Rig, sensorId: string, t: number[], calibVersion: string): Rig {
  return {
    ...rig,
    calib_version: calibVersion,
    sensors: rig.sensors.map((s) => (s.id === sensorId ? { ...s, T_body_sensor: t } : s)),
  };
}

/** `rig` with one GNSS sensor's lever arm set, under `calibVersion`. */
export function withLeverArm(rig: Rig, sensorId: string, leverArmM: Xyz, calibVersion: string): Rig {
  return {
    ...rig,
    calib_version: calibVersion,
    sensors: rig.sensors.map((s) => (s.id === sensorId ? { ...s, lever_arm_m: [...leverArmM] } : s)),
  };
}
