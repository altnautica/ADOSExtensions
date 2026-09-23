/**
 * Wire protocol shared by host and plugin. The plugin host owns the
 * authoritative version in `ADOSMissionControl/src/lib/plugins/types.ts`;
 * this module re-declares the same shape so plugin authors can build
 * without depending on the GCS source tree.
 */

export const PROTOCOL_VERSION = 1 as const;

export type EnvelopeType = "request" | "response" | "event";

export interface RpcEnvelope<TArgs = unknown> {
  id: string;
  type: EnvelopeType;
  method: string;
  capability: string;
  args: TArgs;
  version: typeof PROTOCOL_VERSION;
  error?: { code: string; message: string };
  /** Capability token a request carries (see `capability.token`). */
  token?: string;
}

export interface RpcError {
  code: string;
  message: string;
}

export class HostError extends Error {
  readonly code: string;
  constructor(code: string, message: string) {
    super(message);
    this.code = code;
    this.name = "HostError";
  }
}

/**
 * Telemetry topics the host serves. Each also has a `mavlink.`-prefixed
 * spelling; `battery` carries a normalized {@link BatterySample} while
 * `mavlink.battery` carries the raw adapter frame.
 */
export const TELEMETRY_TOPICS = [
  "attitude",
  "position",
  "battery",
  "gps",
  "vfr",
  "rc",
  "sysStatus",
  "radio",
  "heartbeat",
  "statustext",
  "event",
  "mavlink.attitude",
  "mavlink.position",
  "mavlink.battery",
  "mavlink.gps",
  "mavlink.vfr",
  "mavlink.rc",
  "mavlink.SYS_STATUS",
  "mavlink.radio",
  "mavlink.HEARTBEAT",
  "mavlink.STATUSTEXT",
  "mavlink.EVENT",
] as const;

export type TelemetryTopic = (typeof TELEMETRY_TOPICS)[number];

/**
 * The frame the host delivers on the `battery` telemetry topic. Values the
 * flight controller does not report are `null`; `cellVoltagesV` is empty when
 * per-cell voltages are not reported.
 */
export interface BatterySample {
  timestampMs: number;
  packId: number;
  cellVoltagesV: number[];
  totalVoltageV: number;
  currentA: number | null;
  consumedAh: number | null;
  remainingPercent: number | null;
  temperatureC: number | null;
  cellCount: number | null;
}
