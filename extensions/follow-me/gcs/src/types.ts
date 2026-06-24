/**
 * Shared types for the Follow-Me GCS half.
 *
 * The iframe runs sandboxed with no access to the host's source, so the
 * host-prop and event shapes the host streams in are mirrored here as
 * plain serializable interfaces.
 *
 * @license GPL-3.0-or-later
 */

/** Lock state words shared with the agent + vision contract. */
export type LockState = "locked" | "uncertain" | "lost";

/** The non-gated bridge event the host pushes overlay props on. */
export const VIDEO_OVERLAY_PROPS_EVENT = "video.overlay.props";

/** The topic the agent publishes its follow read-back on (must equal the
 * manifest skill state.topic). */
export const FOLLOW_STATE_TOPIC = "follow.state";

/** The reserved command the overlay sends the operator's designate click on.
 * The host routes it to the vision engine's designate (it locks the tracker
 * onto the clicked box and returns a track id); the agent half then follows
 * whatever the engine has locked. Not an event topic — a host bridge command. */
export const DESIGNATE_COMMAND = "vision.designate";

/** A pixel-space bounding box in the source frame's own resolution. */
export interface BBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

/** One detection in the host-prop shape. */
export interface OverlayDetectionItem {
  bbox: BBox;
  classLabel: string;
  confidence: number;
  trackId: number | null;
  lockState: LockState | null;
}

/** The detection batch carried on the host props, or null when stale. */
export interface OverlayDetections {
  frameWidth: number;
  frameHeight: number;
  frameId: number;
  receivedAt: number;
  items: OverlayDetectionItem[];
}

/** The letterbox-corrected rendered video rect, CSS px relative to the
 * overlay wrapper's top-left. */
export interface RenderedRect {
  left: number;
  top: number;
  width: number;
  height: number;
}

/** Host props pushed to a `video.overlay` iframe. */
export interface VideoOverlayHostProps {
  droneId: string;
  cameraId: string;
  streamWidth: number;
  streamHeight: number;
  renderedRect: RenderedRect;
  frameTimestampMs: number;
  attitude: { rollDeg: number; pitchDeg: number; yawDeg: number };
  detections: OverlayDetections | null;
}

/** The follow read-back the agent publishes on FOLLOW_STATE_TOPIC. */
export interface FollowState {
  active: boolean;
  lockState: LockState | null;
  targetId: number | null;
  rangeM: number | null;
  distanceSetpointM: number | null;
  heightSetpointM: number | null;
  commanding: boolean;
}

/** The agent emits snake_case keys; normalize to the camelCase shape. */
export interface RawFollowState {
  active?: boolean;
  lock_state?: LockState | null;
  target_id?: number | null;
  range_m?: number | null;
  distance_setpoint_m?: number | null;
  height_setpoint_m?: number | null;
  commanding?: boolean;
}

/** The per-drone config the settings tab writes through ctx.config. */
export interface FollowConfig {
  active: boolean;
  follow_distance_m: number;
  follow_height_m: number;
  gimbal_point: boolean;
  designate_camera: string;
  camera_hfov_deg: number;
}

export const DEFAULT_CONFIG: FollowConfig = {
  active: false,
  follow_distance_m: 8,
  follow_height_m: 4,
  gimbal_point: true,
  designate_camera: "uvc-0",
  camera_hfov_deg: 70,
};

/** Empty follow state baseline before the first read-back arrives. */
export const EMPTY_FOLLOW_STATE: FollowState = {
  active: false,
  lockState: null,
  targetId: null,
  rangeM: null,
  distanceSetpointM: null,
  heightSetpointM: null,
  commanding: false,
};

/** Map the agent's snake_case read-back onto the camelCase FollowState. */
export function normalizeFollowState(raw: RawFollowState | null): FollowState {
  if (!raw || typeof raw !== "object") return { ...EMPTY_FOLLOW_STATE };
  return {
    active: raw.active === true,
    lockState: normalizeLockState(raw.lock_state),
    targetId: typeof raw.target_id === "number" ? raw.target_id : null,
    rangeM: typeof raw.range_m === "number" ? raw.range_m : null,
    distanceSetpointM:
      typeof raw.distance_setpoint_m === "number"
        ? raw.distance_setpoint_m
        : null,
    heightSetpointM:
      typeof raw.height_setpoint_m === "number" ? raw.height_setpoint_m : null,
    commanding: raw.commanding === true,
  };
}

function normalizeLockState(v: unknown): LockState | null {
  if (v === "locked" || v === "uncertain" || v === "lost") return v;
  return null;
}
