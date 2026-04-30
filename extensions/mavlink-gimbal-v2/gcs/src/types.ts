/**
 * Shared types for the MAVLink Gimbal v2 controller panel.
 *
 * `GimbalState` mirrors the agent's normalized state struct. `RoiTarget`
 * is the panel-local form payload used when the operator submits the
 * "point at lat/lon/alt" form. The host streams gimbal telemetry on
 * topic `gimbal` per the manifest.
 */

export type GimbalMode = "neutral" | "manual" | "roi-lock" | "follow-me";

export interface GimbalState {
  timestampMs: number;
  pitchDeg: number;
  yawDeg: number;
  rollDeg: number;
  pitchRateDps?: number;
  yawRateDps?: number;
  rollRateDps?: number;
  mode: GimbalMode;
}

export interface RoiTarget {
  latDeg: number;
  lonDeg: number;
  altM: number;
}

export interface AxisLimits {
  pitchMinDeg: number;
  pitchMaxDeg: number;
  yawMinDeg: number;
  yawMaxDeg: number;
  rollMinDeg: number;
  rollMaxDeg: number;
}

export interface PanelOptions {
  vehicleSystemId?: number;
  vehicleComponentId?: number;
  limits?: AxisLimits;
}

export const DEFAULT_LIMITS: AxisLimits = {
  pitchMinDeg: -90,
  pitchMaxDeg: 30,
  yawMinDeg: -180,
  yawMaxDeg: 180,
  rollMinDeg: -45,
  rollMaxDeg: 45,
};

export const DEFAULT_TARGET_SYSTEM_ID = 1;
export const DEFAULT_TARGET_COMPONENT_ID = 154;

export const MAV_CMD_DO_SET_ROI_LOCATION = 195;
export const MAV_CMD_DO_SET_ROI_NONE = 197;
export const MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000;
export const MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001;
