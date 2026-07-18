/**
 * Shared GCS-half types for the SIYI optical-pod plugin.
 *
 * `PodState` mirrors the agent's `siyi.pod.state` read-back (published on the
 * "siyi" telemetry channel and the `siyi.pod.state` event). `PodCapabilities`
 * is the negotiated feature set the console gates its controls on.
 * `VideoOverlayProps` is the host's letterbox-correct overlay contract the
 * video HUD consumes.
 */

export interface PodCapabilities {
  gimbal: boolean;
  zoom: boolean;
  optical_zoom: boolean;
  max_zoom: number;
  thermal: boolean;
  laser: boolean;
  ai_track: boolean;
  sensors: string[];
  streams: string[];
  supports_pip: boolean;
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
  /** Which sensor source each physical leg (main/sub) currently carries. */
  assignment: Record<string, string>;
  yaw_deg: number | null;
  pitch_deg: number | null;
  roll_deg: number | null;
  zoom: number | null;
  gimbal_mode: string;
  palette: number | null;
  recording: boolean;
  laser_range_m: number | null;
  spot_temp_c: number | null;
  track_active: boolean;
  track_id: number | null;
  link_ok: boolean;
  frames_received: number;
}

/** A pixel rect (origin top-left) in the rendered video element's own space. */
export interface RenderedRect {
  x: number;
  y: number;
  width: number;
  height: number;
}

/** The host's `video.overlay.props` payload (letterbox-correct). */
export interface VideoOverlayProps {
  droneId: string;
  cameraId: string | null;
  streamWidth: number;
  streamHeight: number;
  renderedRect: RenderedRect;
  frameTimestampMs: number | null;
  attitude?: { roll: number; pitch: number; yaw: number } | null;
  detections?: OverlayDetection[] | null;
}

export interface OverlayDetection {
  bbox: { x: number; y: number; width: number; height: number };
  trackId?: number | null;
  lockState?: "locked" | "uncertain" | "lost" | null;
  classLabel?: string;
}

export const GIMBAL_MODES = ["lock", "follow", "fpv"] as const;
export type GimbalMode = (typeof GIMBAL_MODES)[number];
