/**
 * @module use-workstation-provisioning
 * @description Keeps every paired drone and ground station holding a
 * credential from every paired workstation, so their lanes to it are
 * admitted, for as long as any World Engine page is open.
 *
 * Provisions a new pair as soon as both nodes are paired and reachable,
 * re-provisions a pair whose credential the workstation no longer lists as
 * current, and retries a failed pair on a fixed interval (see
 * `workstation-provisioning`). One loop runs however many pages are mounted.
 */

import { useEffect } from "react";
import type { InlineHostApi } from "@altnautica/plugin-sdk/inline";

import { useHost } from "../host/context";
import { ComputeAgentClient } from "../lib/agent/compute-client";
import { listNodeCredentials, type ListedNodeCredential } from "../lib/agent/node-credential-client";
import {
  PROVISION_RETRY_MS,
  isComputeProfile,
  planProvisioning,
  provisionAndRecord,
} from "../lib/nodes/workstation-provisioning";
import { useWorkstationLinkStore } from "../stores/workstation-link-store";

/** How long a workstation's credential listing is trusted before a re-read. */
const LISTING_MAX_AGE_MS = 30_000;

let mounts = 0;
let stopLoop: (() => void) | null = null;

function startLoop(host: InlineHostApi): () => void {
  let stopped = false;
  let running = false;
  const listings: Record<string, { at: number; issued: ListedNodeCredential[] | null }> = {};

  const tick = async () => {
    if (running || stopped) return;
    running = true;
    try {
      const nodes = host.nodes.list();
      useWorkstationLinkStore.getState().prune(nodes.map((n) => n.deviceId));
      const peersExist = nodes.some((n) => !isComputeProfile(n.profile));
      for (const ws of nodes) {
        if (!peersExist || !isComputeProfile(ws.profile) || !ws.reachable) continue;
        const cached = listings[ws.deviceId];
        if (cached && Date.now() - cached.at < LISTING_MAX_AGE_MS) continue;
        const client = new ComputeAgentClient(ws.deviceId, host.nodes.agent(ws.deviceId));
        listings[ws.deviceId] = { at: Date.now(), issued: await listNodeCredentials(client) };
      }
      const issued = Object.fromEntries(Object.entries(listings).map(([id, l]) => [id, l.issued]));
      const { links, inFlight } = useWorkstationLinkStore.getState();
      for (const pair of planProvisioning(nodes, links, inFlight, Date.now(), issued)) {
        if (stopped) return;
        const link = await provisionAndRecord(pair, (id) => host.nodes.agent(id));
        // The workstation now lists a new credential; read it again next tick.
        if (link.state === "provisioned") delete listings[pair.workstation.deviceId];
      }
    } finally {
      running = false;
    }
  };

  void tick();
  const id = setInterval(() => void tick(), PROVISION_RETRY_MS);
  return () => {
    stopped = true;
    clearInterval(id);
  };
}

/** Run the shared provisioning loop while the caller is mounted. */
export function useWorkstationProvisioning(): void {
  const host = useHost();
  useEffect(() => {
    mounts += 1;
    if (mounts === 1) stopLoop = startLoop(host);
    return () => {
      mounts -= 1;
      if (mounts === 0) {
        stopLoop?.();
        stopLoop = null;
      }
    };
  }, [host]);
}
