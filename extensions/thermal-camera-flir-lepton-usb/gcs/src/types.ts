/**
 * Shared types for the thermal camera plugin GCS half.
 *
 * `ThermalFrame` is the host-normalised event shape published on the
 * `camera.thermal.frame` topic. The agent ships a lightweight radiometric
 * read-back — the centre-reticle `spot` temperature plus the frame extrema —
 * while the colorized picture rides the video pipeline (the stream leg), so
 * the overlay reads the video leg for the image and this event for the
 * temperatures. A full-frame payload carrying the whole `y16` grid is also
 * accepted (client-side colorize + click-to-spot), so both shapes render.
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
   * counts. Present only on a full-frame payload; the lightweight agent
   * read-back omits it and carries `spot` instead. Hosts that ship
   * structured-clone-safe transports may pass an Array; both shapes are
   * accepted.
   */
  y16?: ArrayLike<number>;
  /**
   * The agent-computed spot temperature at a reticle position. Present on the
   * lightweight read-back (the agent measured the Y16 grid on the drone); the
   * overlay renders this directly rather than reading pixels client-side.
   */
  spot?: { x: number; y: number; temperatureC: number };
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
