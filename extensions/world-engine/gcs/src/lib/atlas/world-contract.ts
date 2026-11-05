/**
 * @module lib/atlas/world-contract
 * @description The GCS half of the World Engine's Atlas wire contract: the
 * topic names, the shared-data capability gate, the bearer keyframe rule, and
 * the framed `AtlasEvent` envelope decoder.
 *
 * Mirrors the extension's agent protocol crate (`world-engine-protocol`),
 * which is the single authority for every constant here, the envelope version
 * included.
 *
 * The agent's own decoder gates the whole contract on the envelope: a frame
 * whose `v` this build does not speak is rejected before its payload is read,
 * because mis-parsing a payload against the wrong schema is worse than dropping
 * it. This mirrors that, and reports WHICH version was refused so an operator
 * surface can name the refused and the spoken version rather than say
 * "malformed".
 */

import { asMsgpackMap, decodeMsgpack, MsgpackError } from "./msgpack";

// --- Topics ----------------------------------------------------------------

/** Capture-session state (state, keyframe counts, VIO health). */
export const ATLAS_CAPTURE_STATE_TOPIC = "atlas.capture.state";
/** A selected pose-tagged keyframe, emitted drone-to-compute. */
export const ATLAS_KEYFRAME_TOPIC = "atlas.keyframe";

/** Shared-data: current 6-DoF navigation pose in `map_ned`. */
export const PLUGIN_ATLAS_POSE_TOPIC = "plugin.world-engine.pose";
/** Shared-data: point-cloud descriptor, or the live sweep-cloud delta. */
export const PLUGIN_ATLAS_POINTCLOUD_TOPIC = "plugin.world-engine.pointcloud";
/** Shared-data: occupancy / ESDF grid descriptor, or the live occupancy tiles. */
export const PLUGIN_ATLAS_OCCUPANCY_TOPIC = "plugin.world-engine.occupancy";
/** Shared-data: gaussian-splat descriptor. */
export const PLUGIN_ATLAS_SPLAT_TOPIC = "plugin.world-engine.splat";
/** Shared-data: mesh descriptor. */
export const PLUGIN_ATLAS_MESH_TOPIC = "plugin.world-engine.mesh";
/** Shared-data: the latest planar scan in the body frame (10 Hz). */
export const PLUGIN_ATLAS_SCAN_TOPIC = "plugin.world-engine.scan";
/** Shared-data: the live map's state (1 Hz). */
export const PLUGIN_ATLAS_MAP_STATUS_TOPIC = "plugin.world-engine.map_status";

/** Every shared-data world-model topic, so a consumer iterates the set rather
 * than hard-coding literals that drift from the agent's own list. */
export const PLUGIN_ATLAS_TOPICS = [
  PLUGIN_ATLAS_POSE_TOPIC,
  PLUGIN_ATLAS_POINTCLOUD_TOPIC,
  PLUGIN_ATLAS_OCCUPANCY_TOPIC,
  PLUGIN_ATLAS_SPLAT_TOPIC,
  PLUGIN_ATLAS_MESH_TOPIC,
  PLUGIN_ATLAS_SCAN_TOPIC,
  PLUGIN_ATLAS_MAP_STATUS_TOPIC,
] as const;

/** The four ARTIFACT topics — the world model proper, excluding the live pose
 * lane, which is telemetry rather than an artifact. */
export const PLUGIN_ATLAS_ARTIFACT_TOPICS = [
  PLUGIN_ATLAS_SPLAT_TOPIC,
  PLUGIN_ATLAS_POINTCLOUD_TOPIC,
  PLUGIN_ATLAS_MESH_TOPIC,
  PLUGIN_ATLAS_OCCUPANCY_TOPIC,
] as const;

// --- Shared-data access policy --------------------------------------------

/** The capability gating the world-model artifact descriptors: the
 * extension's own declared capability, since a descriptor names where a
 * reconstruction of the flown area can be fetched. */
export const ATLAS_WORLD_READ_CAP = "plugin.world-engine.world.read";

/** The capability gating the live ~10 Hz world pose — the same class of data as
 * the vehicle telemetry a consumer already reads, in the world frame. */
export const ATLAS_POSE_READ_CAP = "telemetry.read";

/**
 * The capability required to subscribe to `topic`, or null when `topic` is not a
 * world-model shared-data topic.
 *
 * **Exact match, never a prefix.** `plugin.world-engine.occupancy.evil`
 * resolves to null so it inherits nothing from `plugin.world-engine.occupancy`;
 * a prefix match would let a caller mint a topic under a gated prefix and be
 * handed that prefix's capability, which turns the gate into a naming
 * convention.
 */
export function atlasTopicSubscribeCapability(topic: string): string | null {
  if (
    topic === PLUGIN_ATLAS_POSE_TOPIC ||
    topic === PLUGIN_ATLAS_SCAN_TOPIC ||
    topic === PLUGIN_ATLAS_MAP_STATUS_TOPIC
  ) {
    return ATLAS_POSE_READ_CAP;
  }
  if (
    topic === PLUGIN_ATLAS_POINTCLOUD_TOPIC ||
    topic === PLUGIN_ATLAS_OCCUPANCY_TOPIC ||
    topic === PLUGIN_ATLAS_SPLAT_TOPIC ||
    topic === PLUGIN_ATLAS_MESH_TOPIC
  ) {
    return ATLAS_WORLD_READ_CAP;
  }
  return null;
}

// --- Live navigation and mapping payloads ----------------------------------
//
// Frames: `map_ned` is north-aligned (x north, y east, z down, metres): the
// planar SLAM world rotated by the FC's compass heading latched at the first
// pose, origin at the first scan, then moved onto the FC's LOCAL_POSITION_NED
// (yaw and offset fitted against the EKF's GNSS track) while the FC navigates
// on GNSS, frozen when GNSS goes. `body_frd` is the vehicle body,
// forward-right-down.

export type NavFrame = "map_ned" | "body_frd";
export type NavHealth = "good" | "degraded" | "lost";
export type NavPoseSource = "lidar_planar" | "fc_ekf" | "vio" | "replay";
export type ComputePlacement = "onboard" | "stream" | "post_flight" | "hybrid";
export type NavPoseMode = "always" | "capture";

/** `plugin.world-engine.pose`: `R` row-major world-from-body, `t` metres. */
export interface NavPose {
  pose: { R: number[]; t: [number, number, number]; cov: number[] | null };
  anchor: unknown | null;
  ts_ms: number;
  health: NavHealth;
  frame: NavFrame;
  source: NavPoseSource;
}

/** `plugin.world-engine.scan`: `ranges_m[i]` at `angle_start_rad +
 * i * angle_increment_rad`, clockwise from body forward; `0` = no return. */
export interface NavScan {
  t_ns: number;
  frame: "body_frd";
  angle_start_rad: number;
  angle_increment_rad: number;
  ranges_m: number[];
}

/** Live occupancy cell values (ROS map_server convention). */
export const OCCUPANCY_OCCUPIED = 0;
export const OCCUPANCY_FREE = 254;
export const OCCUPANCY_UNKNOWN = 205;

/** One changed tile: `tile_cells` square grey PNG, row 0 = largest x (north
 * up), column 0 = smallest y (west left); covers `[ix, ix+1) * tile_cells *
 * res_m` in x and likewise in y. */
export interface OccupancyTile {
  ix: number;
  iy: number;
  png_b64: string;
}

/** `plugin.world-engine.occupancy` live delta (1 Hz, changed tiles only). */
export interface LiveOccupancy {
  kind: "live";
  seq: number;
  frame: "map_ned";
  res_m: number;
  tile_cells: number;
  tiles: OccupancyTile[];
}

/** `plugin.world-engine.pointcloud` live delta: `count` little-endian f32
 * xyz triples. A changed `voxel_m` supersedes every earlier delta. */
export interface LivePointCloud {
  kind: "live";
  seq: number;
  frame: "map_ned";
  voxel_m: number;
  count: number;
  points_b64: string;
}

/** `plugin.world-engine.map_status` (1 Hz). */
export interface MapStatus {
  session_id: string | null;
  placement: ComputePlacement;
  memory_used_mb: number;
  memory_budget_mb: number;
  voxel_m: number;
  keyframes_dropped: number;
  sweep_points: number;
  area_m2: number;
  nav_pose: NavPoseMode;
  upload: {
    state: "idle" | "queued" | "uploading" | "done" | "failed";
    bytes_done: number;
    bytes_total: number;
  };
  stream: { state: "off" | "connecting" | "live" | "degraded"; bytes_per_s: number };
}

/** Whether a payload on a topic that also carries node products is live. A
 * product descriptor says `kind: "product"`. */
export function isLiveMapPayload(payload: unknown): boolean {
  return (
    typeof payload === "object" &&
    payload !== null &&
    (payload as { kind?: unknown }).kind === "live"
  );
}

// --- Bearer rule -----------------------------------------------------------

/**
 * Whether a bearer can carry a full keyframe, from the bearer name the
 * forwarder reports (`direct-lan` / `wfb-relay` / `cloud`).
 *
 * A pure property of the bearer, not a runtime measurement: the WFB relay's
 * datagram ceiling is 1300 bytes against a multi-MB JPEG keyframe, so a no-LAN
 * field topology degrades to pose and status only — i.e. NO world model — while
 * the operator surface would otherwise report `bearer: "wfb-relay"` as though
 * the lane were working.
 */
export function bearerCarriesKeyframes(bearer: string): boolean {
  return bearer !== "wfb-relay";
}

/** True when the bearer cannot carry keyframes, so the world-model lane is
 * degraded to pose and status. The operator-facing wording is a locale string
 * on this side rather than the agent's verbatim English. */
export function bearerKeyframesDegraded(bearer: string | null): boolean {
  return bearer !== null && !bearerCarriesKeyframes(bearer);
}

// --- Envelope --------------------------------------------------------------

/** The on-wire envelope version this build speaks. */
export const ATLAS_ENVELOPE_VERSION = 2;

/** One framed message off the atlas bus / world-descriptor stream. */
export interface AtlasEvent {
  /** Envelope version, always {@link ATLAS_ENVELOPE_VERSION} once decoded. */
  v: number;
  topic: string;
  /** The capturing drone's device id, stamped by the drone-side forwarder on
   * egress. Null on a local-bus event that never left the drone. */
  deviceId: string | null;
  /** The topic's own msgpack-encoded struct, still undecoded. */
  payload: Uint8Array;
}

/** Why an envelope was refused. Distinct reasons because they mean different
 * things to an operator: `version` is a build/agent mismatch to act on, while
 * `malformed` is a corrupt or non-contract frame. */
export type AtlasEventRejection = "malformed" | "shape" | "version";

export type AtlasEventDecodeResult =
  | { ok: true; event: AtlasEvent }
  | {
      ok: false;
      reason: AtlasEventRejection;
      detail: string;
      /** The refused envelope version, when the frame decoded far enough to
       * carry one. */
      version: number | null;
    };

/** `Vec<u8>` crosses as a msgpack array of integers (see the msgpack module),
 * so the payload is rebuilt from that array. A `bin` payload is also accepted
 * so a producer that later adopts `serde_bytes` needs no change here. */
function payloadBytes(value: unknown): Uint8Array | null {
  if (value instanceof Uint8Array) return value;
  if (!Array.isArray(value)) return null;
  const out = new Uint8Array(value.length);
  for (let i = 0; i < value.length; i++) {
    const b = value[i];
    if (typeof b !== "number" || !Number.isInteger(b) || b < 0 || b > 255) {
      return null;
    }
    out[i] = b;
  }
  return out;
}

/**
 * Decode a framed `AtlasEvent`, refusing a frame whose envelope version this
 * build does not speak before its payload is touched.
 *
 * Returns a result rather than throwing: the stream consumer counts refusals by
 * reason and keeps streaming, exactly as the agent's own fan-out skips a lagged
 * descriptor instead of stalling the trainer.
 */
export function decodeAtlasEvent(bytes: Uint8Array): AtlasEventDecodeResult {
  let map: { [key: string]: unknown } | null;
  try {
    map = asMsgpackMap(decodeMsgpack(bytes));
  } catch (err) {
    return {
      ok: false,
      reason: "malformed",
      detail: err instanceof MsgpackError ? err.message : String(err),
      version: null,
    };
  }
  if (!map) {
    return {
      ok: false,
      reason: "shape",
      detail: "envelope is not a msgpack map",
      version: null,
    };
  }

  // The version field rides as `v` and carries no serde default, so a frame
  // without it is not an envelope at all.
  const v = map.v;
  if (typeof v !== "number" || !Number.isInteger(v)) {
    return {
      ok: false,
      reason: "shape",
      detail: "envelope carries no integer version field",
      version: null,
    };
  }
  if (v !== ATLAS_ENVELOPE_VERSION) {
    return {
      ok: false,
      reason: "version",
      detail: `unsupported atlas envelope version ${v} (this build speaks ${ATLAS_ENVELOPE_VERSION})`,
      version: v,
    };
  }

  const topic = map.topic;
  if (typeof topic !== "string" || topic.length === 0) {
    return {
      ok: false,
      reason: "shape",
      detail: "envelope carries no topic",
      version: v,
    };
  }
  const payload = payloadBytes(map.payload);
  if (!payload) {
    return {
      ok: false,
      reason: "shape",
      detail: "envelope payload is not a byte sequence",
      version: v,
    };
  }
  const deviceId = map.device_id;
  return {
    ok: true,
    event: {
      v,
      topic,
      deviceId: typeof deviceId === "string" && deviceId.length > 0 ? deviceId : null,
      payload,
    },
  };
}
