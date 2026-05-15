/**
 * Shared types for the vision-nav GCS plugin. The shape of
 * VisionNavTelemetry mirrors the agent heartbeat's navigation
 * capability block: optical flow stats, optional VIO stats, and the
 * companion process state. The agent emits these as a single object
 * on the "navigation" telemetry topic.
 */

export interface VisionNavTelemetry {
  flowQuality?: number;
  flowRateHz?: number;
  flowDistanceM?: number | null;
  vioState?: "active" | "degraded" | "lost" | "absent";
  vioResetCounter?: number;
  vioQuality?: number;
  companionState?: "active" | "critical" | "terminating" | "absent";
  opticalFlowSupported: boolean;
  vioSupported: boolean;
}

export type FirmwareType = "ardupilot" | "px4" | "betaflight" | "inav";

export type EkfSourceSet = 1 | 2 | 3;

export interface EkfSourceOption {
  set: EkfSourceSet;
  label: string;
  description: string;
}
