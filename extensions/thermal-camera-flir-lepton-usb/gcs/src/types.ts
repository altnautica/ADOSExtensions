/**
 * Shared types for the thermal camera plugin GCS half.
 *
 * `ThermalFrame` is the host-normalised event shape published on the
 * `camera.thermal.frame` topic. The agent half computes Y16 plus a
 * matching kelvin grid; the GCS half receives both and renders.
 */

export type PaletteName = "ironbow" | "rainbow" | "grayscale";

export const PALETTE_NAMES: ReadonlyArray<PaletteName> = [
  "ironbow",
  "rainbow",
  "grayscale",
];

export type AgcMode = "linear" | "histogram" | "fixed";

export interface ThermalFrame {
  /** Monotonic timestamp from the agent's frame loop. */
  timestampNs: number;
  /** Sequence number from the driver. */
  sequence: number;
  /** Frame width in pixels. */
  width: number;
  /** Frame height in pixels. */
  height: number;
  /**
   * Flat ``Uint16Array`` of length ``width * height`` carrying raw Y16
   * counts. Hosts that ship structured-clone-safe transports may pass
   * an Array; both shapes are accepted.
   */
  y16: ArrayLike<number>;
  /** Per-frame extrema in deg C, as reported by the agent. Optional. */
  minC?: number;
  maxC?: number;
  /** Default tlinear resolution (K/count). Defaults to 0.01. */
  resolutionKPerCount?: number;
}

export interface SpotMeterState {
  /** X column in the frame's coordinate system. */
  x: number;
  /** Y row in the frame's coordinate system. */
  y: number;
  /** Most recent reading in deg C, or ``null`` until the first frame. */
  temperatureC: number | null;
}

export interface IsothermConfig {
  enabled: boolean;
  lowerC: number;
  upperC: number;
}

export interface AlarmConfig {
  enabled: boolean;
  thresholdC: number;
}

export interface FfcConfig {
  autoOnDisarm: boolean;
  autoIntervalMinutes: number;
}

export interface FixedRangeConfig {
  minC: number;
  maxC: number;
}

export interface ThermalCameraConfig {
  palette: PaletteName;
  agc: AgcMode;
  fixedRange: FixedRangeConfig;
  spotMeter: { x: number; y: number };
  isotherm: IsothermConfig;
  alarm: AlarmConfig;
  ffc: FfcConfig;
}

export const DEFAULT_THERMAL_CONFIG: ThermalCameraConfig = {
  palette: "ironbow",
  agc: "linear",
  fixedRange: { minC: 0, maxC: 100 },
  spotMeter: { x: 80, y: 60 },
  isotherm: { enabled: false, lowerC: 30, upperC: 60 },
  alarm: { enabled: false, thresholdC: 80 },
  ffc: { autoOnDisarm: true, autoIntervalMinutes: 5 },
};

export const KELVIN_C_OFFSET = 273.15;
export const DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT = 0.01;
