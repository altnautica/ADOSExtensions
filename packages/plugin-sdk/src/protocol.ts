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

/** Shared topic taxonomy used to derive capability strings. */
export const TELEMETRY_TOPICS = [
  "battery",
  "mavlink",
  "mavlink.STATUSTEXT",
  "mavlink.HEARTBEAT",
  "mavlink.SYS_STATUS",
  "video.stats",
] as const;

export type TelemetryTopic = (typeof TELEMETRY_TOPICS)[number];
