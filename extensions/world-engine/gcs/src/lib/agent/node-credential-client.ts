/**
 * @module node-credential-client
 * @description Moving a workstation-issued credential onto the nodes that use
 * the workstation's lanes.
 *
 * Each node mints its own pairing key, so a drone cannot authenticate to a
 * workstation with its own key, and must never hand that key to one. The GCS
 * is the operator of both, so it asks the workstation to issue the drone a
 * credential scoped to the lanes a drone uses (the Atlas ingest and world
 * stream, the offload session and its detection stream, the artifacts), and
 * installs it on the drone. A ground station, which only relays Atlas events,
 * is issued the ingest lane alone.
 *
 * Workstation side (its job API, through {@link ComputeAgentClient}): issue,
 * list, revoke. Drone / ground-station side (the extension's own server on
 * that node): install.
 */

import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

import type { ComputeAgentClient } from "./compute-client";
import { requestJson, type JsonReply } from "./plugin-http";

/** The lanes a credential can be scoped to (the agent's wire names). */
export type NodeLane =
  | "atlas.ingest"
  | "atlas.world"
  | "offload.stream"
  | "artifacts.read"
  | "jobs.submit";

export const DRONE_LANES: readonly NodeLane[] = [
  "atlas.ingest",
  "atlas.world",
  "offload.stream",
  "artifacts.read",
  "jobs.submit",
];

/** A ground station only relays Atlas capture events off the air. */
export const GROUND_STATION_LANES: readonly NodeLane[] = ["atlas.ingest"];

/** A credential a workstation just issued; `credential` is shown once. */
export interface IssuedNodeCredential {
  id: string;
  peerDeviceId: string;
  lanes: NodeLane[];
  createdAtMs: number;
  credential: string;
  /** The id the workstation advertises over mDNS; the drone files the
   * credential under it so each lane presents it only to that workstation. */
  workstationNodeId: string;
}

/** One credential as the workstation lists it (never the secret). */
export interface ListedNodeCredential {
  id: string;
  peerDeviceId: string;
  lanes: NodeLane[];
  createdAtMs: number;
  /** Issued under the workstation's current owner key; false once the
   * workstation was re-paired, when it admits nothing. */
  current: boolean;
}

/** A failed call: the HTTP status (0 = unreachable) and the agent's reason. */
export interface CallFailure {
  ok: false;
  status: number;
  message: string;
}

function lanesOf(v: unknown): NodeLane[] {
  const known: readonly string[] = DRONE_LANES;
  return Array.isArray(v)
    ? v.filter((l): l is NodeLane => typeof l === "string" && known.includes(l))
    : [];
}

function failure(r: JsonReply | null): CallFailure {
  if (!r) return { ok: false, status: 0, message: "unreachable" };
  const body = r.json as Record<string, unknown> | null;
  const reason = body?.error ?? body?.detail;
  return {
    ok: false,
    status: r.status,
    message: typeof reason === "string" ? reason : `HTTP ${r.status}`,
  };
}

/** Ask the workstation to issue `peerDeviceId` a credential for `lanes`,
 * replacing any it issued that peer before. */
export async function issueNodeCredential(
  workstation: ComputeAgentClient,
  peerDeviceId: string,
  lanes: readonly NodeLane[],
): Promise<({ ok: true } & IssuedNodeCredential) | CallFailure> {
  const r = await workstation.jobCall("node-credentials", "POST", {
    peer_device_id: peerDeviceId,
    lanes,
  });
  const b = r?.json as Record<string, unknown> | null | undefined;
  if (
    !r ||
    r.status !== 201 ||
    typeof b?.credential !== "string" ||
    typeof b.id !== "string" ||
    typeof b.workstation_node_id !== "string"
  ) {
    return failure(r);
  }
  return {
    ok: true,
    id: b.id,
    peerDeviceId: typeof b.peer_device_id === "string" ? b.peer_device_id : peerDeviceId,
    lanes: lanesOf(b.lanes),
    createdAtMs: typeof b.created_at_ms === "number" ? b.created_at_ms : 0,
    credential: b.credential,
    workstationNodeId: b.workstation_node_id,
  };
}

/** Every credential the workstation issued, or `null` when it cannot be read. */
export async function listNodeCredentials(
  workstation: ComputeAgentClient,
): Promise<ListedNodeCredential[] | null> {
  const r = await workstation.jobCall("node-credentials", "GET");
  const b = r?.json as Record<string, unknown> | null | undefined;
  if (!r || r.status !== 200 || !Array.isArray(b?.credentials)) return null;
  return b.credentials.flatMap((raw): ListedNodeCredential[] => {
    const c = raw as Record<string, unknown>;
    if (typeof c?.id !== "string" || typeof c.peer_device_id !== "string") return [];
    return [
      {
        id: c.id,
        peerDeviceId: c.peer_device_id,
        lanes: lanesOf(c.lanes),
        createdAtMs: typeof c.created_at_ms === "number" ? c.created_at_ms : 0,
        current: c.current === true,
      },
    ];
  });
}

/** Revoke one credential on the workstation. */
export async function revokeNodeCredential(
  workstation: ComputeAgentClient,
  id: string,
): Promise<{ ok: true } | CallFailure> {
  const r = await workstation.jobCall(
    `node-credentials/${encodeURIComponent(id)}/revoke`,
    "POST",
  );
  const b = r?.json as Record<string, unknown> | null | undefined;
  return r && r.status === 200 && b?.revoked === true ? { ok: true } : failure(r);
}

/** Install a workstation-issued credential on a drone or ground station,
 * through the extension's server on that node (`node-credential`). */
export async function installWorkstationCredential(
  peer: InlineAgentApi,
  issued: Pick<IssuedNodeCredential, "workstationNodeId" | "credential" | "lanes">,
): Promise<{ ok: true } | CallFailure> {
  const r = await requestJson(peer, "node-credential", "POST", {
    workstation_node_id: issued.workstationNodeId,
    credential: issued.credential,
    lanes: issued.lanes,
  });
  const b = r?.json as Record<string, unknown> | null | undefined;
  return r && r.status === 200 && b?.installed === true ? { ok: true } : failure(r);
}
