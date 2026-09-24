/**
 * @module workstation-provisioning
 * @description Which paired nodes need a workstation credential, and the one
 * act that gives them one.
 *
 * Whenever this GCS holds a paired workstation (or compute node) and a paired
 * drone or ground station, the drone needs a credential from that workstation
 * for its lanes (see `node-credential-client`). {@link planProvisioning} is the
 * pure decision over the paired nodes, the recorded links and what each
 * workstation says it issued; {@link provisionAndRecord} issues on the
 * workstation, installs on the peer, and records the outcome. A failed link is
 * retried on a fixed interval with no cap; a revoked one only when the
 * operator asks.
 */

import type { InlineAgentApi, InlineNodeSummary } from "@altnautica/plugin-sdk/inline";

import { ComputeAgentClient } from "../agent/compute-client";
import {
  DRONE_LANES,
  GROUND_STATION_LANES,
  installWorkstationCredential,
  issueNodeCredential,
  type ListedNodeCredential,
  type NodeLane,
} from "../agent/node-credential-client";
import {
  useWorkstationLinkStore,
  workstationLinkKey,
  type WorkstationLink,
} from "../../stores/workstation-link-store";

/** Fixed interval between attempts on a link that failed. */
export const PROVISION_RETRY_MS = 5000;

/** One workstation-to-peer link to provision. */
export interface ProvisionPair {
  workstation: InlineNodeSummary;
  peer: InlineNodeSummary;
  lanes: readonly NodeLane[];
}

/** Whether a node of `profile` runs the compute node (issues credentials). */
export function isComputeProfile(profile: InlineNodeSummary["profile"]): boolean {
  return profile === "workstation" || profile === "compute";
}

/** The lanes a node of `profile` is issued, or `null` for a profile that uses
 * no workstation lane. */
export function lanesForPeer(profile: InlineNodeSummary["profile"]): readonly NodeLane[] | null {
  switch (profile) {
    case "drone":
      return DRONE_LANES;
    case "ground-station":
      return GROUND_STATION_LANES;
    case "workstation":
    case "compute":
      return null;
  }
}

/**
 * Every workstation-to-peer link that needs provisioning now: never
 * provisioned; provisioned, but the workstation no longer lists that
 * credential as current (it was re-paired or wiped); or failed at least
 * {@link PROVISION_RETRY_MS} ago. A revoked link and one already in flight are
 * skipped, as is any node this browser cannot reach.
 *
 * @param issued What each workstation last listed, by its device id; absent
 *   or null when not read (then a provisioned link is trusted).
 */
export function planProvisioning(
  nodes: readonly InlineNodeSummary[],
  links: Readonly<Record<string, WorkstationLink>>,
  inFlight: Readonly<Record<string, true>>,
  now: number,
  issued: Readonly<Record<string, readonly ListedNodeCredential[] | null>> = {},
): ProvisionPair[] {
  const pairs: ProvisionPair[] = [];
  for (const workstation of nodes) {
    if (!isComputeProfile(workstation.profile) || !workstation.reachable) continue;
    const listing = issued[workstation.deviceId];
    for (const peer of nodes) {
      const lanes = lanesForPeer(peer.profile);
      if (!lanes || peer.deviceId === workstation.deviceId || !peer.reachable) continue;
      const key = workstationLinkKey(workstation.deviceId, peer.deviceId);
      if (inFlight[key]) continue;
      const link = links[key];
      const stale =
        link?.state === "provisioned" &&
        Array.isArray(listing) &&
        !listing.some((c) => c.id === link.credentialId && c.current);
      const due =
        link === undefined ||
        stale ||
        (link.state === "failed" && now - link.at >= PROVISION_RETRY_MS);
      if (due) pairs.push({ workstation, peer, lanes });
    }
  }
  return pairs;
}

/**
 * Issue a credential for `pair.peer` on the workstation, install it on the
 * peer, and record the outcome. One attempt per pair at a time; a pair already
 * in flight is left alone.
 *
 * @param agentFor The extension's server on a node, by device id.
 */
export async function provisionAndRecord(
  pair: ProvisionPair,
  agentFor: (deviceId: string) => InlineAgentApi,
): Promise<WorkstationLink> {
  const { workstation, peer, lanes } = pair;
  const key = workstationLinkKey(workstation.deviceId, peer.deviceId);
  const store = useWorkstationLinkStore.getState();
  const base = { workstationDeviceId: workstation.deviceId, peerDeviceId: peer.deviceId };
  if (store.inFlight[key]) {
    return store.links[key] ?? { ...base, state: "failed", at: Date.now(), error: "in progress" };
  }
  store.setInFlight(key, true);
  let link: WorkstationLink;
  try {
    const issued = await issueNodeCredential(
      new ComputeAgentClient(workstation.deviceId, agentFor(workstation.deviceId)),
      peer.deviceId,
      lanes,
    );
    if (!issued.ok) {
      link = { ...base, state: "failed", at: Date.now(), error: `workstation: ${issued.message}` };
    } else {
      const installed = await installWorkstationCredential(agentFor(peer.deviceId), issued);
      link = installed.ok
        ? { ...base, state: "provisioned", credentialId: issued.id, at: Date.now() }
        : {
            ...base,
            state: "failed",
            credentialId: issued.id,
            at: Date.now(),
            error: `${peer.name}: ${installed.message}`,
          };
    }
  } catch (err) {
    // A host refusal (the node became unreachable mid-attempt) is a failed
    // attempt like any other, retried on the interval.
    link = {
      ...base,
      state: "failed",
      at: Date.now(),
      error: err instanceof Error ? err.message : String(err),
    };
  } finally {
    useWorkstationLinkStore.getState().setInFlight(key, false);
  }
  useWorkstationLinkStore.getState().record(link);
  return link;
}
