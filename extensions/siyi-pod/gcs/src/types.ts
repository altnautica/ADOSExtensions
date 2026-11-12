/**
 * Shared GCS-half types for the SIYI optical-pod plugin.
 *
 * `PodState` mirrors the agent's `siyi.pod.state` read-back (published on the
 * "siyi" telemetry channel and the `siyi.pod.state` event). `PodCapabilities`
 * is the negotiated feature set the console gates its controls on. The video
 * HUD consumes the SDK's `VideoOverlayHostProps`.
 */

export interface PodCapabilities {
  gimbal: boolean;
  zoom: boolean;
  optical_zoom: boolean;
  max_zoom: number;
  thermal: boolean;
  laser: boolean;
  sensors: string[];
  yaw_min: number;
  yaw_max: number;
  pitch_min: number;
  pitch_max: number;
}

export interface PodState {
  model: string;
  known: boolean;
  connected: boolean;
  firmware: string | null;
  capabilities: Partial<PodCapabilities>;
  /** Which sensor each physical leg (main/sub) carries. */
  assignment: Record<string, string>;
  yaw_deg: number | null;
  pitch_deg: number | null;
  roll_deg: number | null;
  zoom: number | null;
  gimbal_mode: string;
  palette: number | null;
  recording: boolean;
  laser_range_m: number | null;
  /** When the range was measured (epoch ms); null when never measured. */
  laser_range_at: number | null;
  /** Whether the operator has armed the rangefinder. */
  laser_enabled: boolean;
  spot_temp_c: number | null;
  link_ok: boolean;
  frames_received: number;
}

export const GIMBAL_MODES = ["lock", "follow", "fpv"] as const;

/** Thermal palette codes the pod accepts (code 1 is reserved by the pod). */
export const PALETTE_CODES = [0, 2, 3, 4, 5, 6, 7] as const;
export type GimbalMode = (typeof GIMBAL_MODES)[number];
