/**
 * @module vision/offload-target
 * @description Maps a paired workstation node to the `host:port` address a
 * drone stores as its perception-offload target (`offload.compute_node_addr`).
 * An empty string means auto-discover (the drone picks any serving
 * workstation on the LAN).
 */

import { COMPUTE_JOB_PORT } from "../agent/compute-client";

/**
 * The address the drone dials for offload: the workstation's LAN host plus
 * the compute node's own job-API port (`:8092`), NOT the agent front the GCS
 * paired to. The offload path submits jobs to the compute node, which listens
 * on its own port. Empty when the node has no LAN host.
 */
export function nodeToOffloadAddr(node: { lanHost: string | null }): string {
  const host = node.lanHost?.trim();
  return host ? `${host}:${COMPUTE_JOB_PORT}` : "";
}
