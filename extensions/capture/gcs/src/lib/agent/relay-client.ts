/**
 * @module agent/relay-client
 * @description capture keyframe relay status on a ground-station node.
 *
 * The ground station bridges the drone's WFB aux lane (small capture events)
 * onto the LAN, forwarding each decoded keyframe datagram to the compute
 * node. This read surfaces the relay's received-side counters so the
 * operator sees the field lane is delivering. A node that is not a relay
 * answers 404; a transport failure or any other non-2xx is "unreachable",
 * never "no relay".
 */

import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

import { requestJson } from "@altnautica/mapping-ui/lib/agent/plugin-http";

/** Path of the relay status route on the extension's ground-station server. */
export const RELAY_STATUS_PATH = "capture/relay/status";

/** Counters and config for the ground-station capture keyframe relay. With no
 * current snapshot the agent nulls every key under `stale: true`; those
 * readings stay null here rather than reading as a stopped relay with zero
 * counters. */
export interface CaptureRelayStatus {
  /** Whether a relay loop is currently running on this node; null when stale. */
  up: boolean | null;
  /** Datagrams read off the decoded aux port (received-side liveness proof). */
  datagramsSeen: number | null;
  /** Events decoded and accepted by the compute receiver. */
  forwarded: number | null;
  /** Datagrams that did not decode to an capture event (dropped). */
  malformed: number | null;
  /** Events that decoded but the forward POST to the compute node failed. */
  forwardFailed: number | null;
  /** The compute node base URL the relay forwards to. */
  computeUrl: string | null;
  /** The loopback port the aux stream is decoded onto. */
  listenPort: number | null;
  /** The agent has no current snapshot from the relay loop (it age-gates the
   * snapshot on its own clock, so no cross-host clock comparison is needed). */
  stale: boolean;
}

/** The outcome of one relay-status read. */
export type CaptureRelayRead =
  | { kind: "snapshot"; status: CaptureRelayStatus }
  /** The node answered 404: it runs no capture relay. */
  | { kind: "absent" }
  /** Transport failure, timeout, or a non-2xx other than 404. */
  | { kind: "unreachable" };

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

/** Coerce the route's JSON body; null when it is not an object. */
export function coerceRelayStatus(raw: unknown): CaptureRelayStatus | null {
  if (!raw || typeof raw !== "object") return null;
  const e = raw as Record<string, unknown>;
  return {
    up: typeof e.up === "boolean" ? e.up : null,
    datagramsSeen: num(e.datagrams_seen),
    forwarded: num(e.forwarded),
    malformed: num(e.malformed),
    forwardFailed: num(e.forward_failed),
    computeUrl: typeof e.compute_url === "string" ? e.compute_url : null,
    listenPort: num(e.listen_port),
    stale: e.stale === true,
  };
}

/** Read the ground-station capture relay status through the node's agent. */
export async function getRelayStatus(agent: InlineAgentApi): Promise<CaptureRelayRead> {
  const reply = await requestJson(agent, RELAY_STATUS_PATH, "GET");
  if (!reply) return { kind: "unreachable" };
  if (reply.status === 404) return { kind: "absent" };
  if (reply.status < 200 || reply.status >= 300) return { kind: "unreachable" };
  const status = coerceRelayStatus(reply.json);
  return status ? { kind: "snapshot", status } : { kind: "unreachable" };
}
