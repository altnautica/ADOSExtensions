/**
 * What the `lidar-slam` service publishes for the Agent page: its map
 * status (`plugin.navigation.map_status`) and the LiDAR link
 * (`plugin.navigation.lidar`), both snake_case on the wire.
 */

export const MAP_STATUS_TOPIC = "plugin.navigation.map_status";
export const LIDAR_TOPIC = "plugin.navigation.lidar";

export type Placement = "onboard" | "stream" | "post_flight" | "hybrid";
export type PoseProvider = "lidar_planar" | "fc_ekf" | "vio" | "replay";

export interface RawMapStatus {
  placement?: string;
  memory_used_mb?: number | null;
  memory_budget_mb?: number;
  voxel_m?: number;
  sweep_points?: number;
  area_m2?: number;
  pose_provider?: string | null;
}

export interface MapStatus {
  placement: Placement | null;
  memoryUsedMb: number | null;
  memoryBudgetMb: number | null;
  voxelM: number | null;
  sweepPoints: number | null;
  areaM2: number | null;
  poseProvider: PoseProvider | null;
}

export const EMPTY_MAP_STATUS: MapStatus = {
  placement: null,
  memoryUsedMb: null,
  memoryBudgetMb: null,
  voxelM: null,
  sweepPoints: null,
  areaM2: null,
  poseProvider: null,
};

const PLACEMENTS: readonly Placement[] = ["onboard", "stream", "post_flight", "hybrid"];
const PROVIDERS: readonly PoseProvider[] = ["lidar_planar", "fc_ekf", "vio", "replay"];

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function oneOf<T extends string>(v: unknown, set: readonly T[]): T | null {
  return typeof v === "string" && (set as readonly string[]).includes(v) ? (v as T) : null;
}

export function normalizeMapStatus(raw: RawMapStatus | null | undefined): MapStatus {
  if (!raw) return EMPTY_MAP_STATUS;
  return {
    placement: oneOf(raw.placement, PLACEMENTS),
    memoryUsedMb: num(raw.memory_used_mb),
    memoryBudgetMb: num(raw.memory_budget_mb),
    voxelM: num(raw.voxel_m),
    sweepPoints: num(raw.sweep_points),
    areaM2: num(raw.area_m2),
    poseProvider: oneOf(raw.pose_provider, PROVIDERS),
  };
}

export interface RawLidarStatus {
  device?: string | null;
  connected?: boolean;
  health?: string | null;
  rate_hz?: number | null;
  returns_last_scan?: number | null;
  scans?: number;
  reconnects?: number;
}

export interface LidarStatus {
  device: string | null;
  connected: boolean | null;
  health: "good" | "warning" | "error" | null;
  rateHz: number | null;
  returnsLastScan: number | null;
  scans: number | null;
  reconnects: number | null;
}

export const EMPTY_LIDAR_STATUS: LidarStatus = {
  device: null,
  connected: null,
  health: null,
  rateHz: null,
  returnsLastScan: null,
  scans: null,
  reconnects: null,
};

export function normalizeLidarStatus(raw: RawLidarStatus | null | undefined): LidarStatus {
  if (!raw) return EMPTY_LIDAR_STATUS;
  return {
    device: typeof raw.device === "string" ? raw.device : null,
    connected: typeof raw.connected === "boolean" ? raw.connected : null,
    health: oneOf(raw.health, ["good", "warning", "error"] as const),
    rateHz: num(raw.rate_hz),
    returnsLastScan: num(raw.returns_last_scan),
    scans: num(raw.scans),
    reconnects: num(raw.reconnects),
  };
}
