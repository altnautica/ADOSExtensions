/**
 * What each mapping extension provisions from a reconstruction node onto the
 * devices it runs on: the lanes its processes present to the node, and the
 * route that installs a credential into its own store.
 */

import type { InlineNodeSummary } from "@altnautica/plugin-sdk/inline";

import { RECONSTRUCT_PLUGIN } from "../../host/context";
import type { NodeLane } from "../agent/node-credential-client";
import type { CredentialTarget } from "./workstation-provisioning";

/** The extension that captures sessions and forwards keyframes. */
export const CAPTURE_PLUGIN = "com.altnautica.capture";

/** Capture on a drone or Visor: keyframes to the node, session uploads,
 * reconstruction submits. A ground station only relays capture events off
 * the air. */
export function captureLanes(profile: InlineNodeSummary["profile"]): readonly NodeLane[] | null {
  switch (profile) {
    case "drone":
    case "visor":
      return ["atlas.ingest", "sessions.upload", "jobs.submit"];
    case "ground-station":
      return ["atlas.ingest"];
    default:
      return null;
  }
}

/** Reconstruct on a drone: perception offload, the live scan stream and
 * reconstruction submits. */
export function reconstructLanes(profile: InlineNodeSummary["profile"]): readonly NodeLane[] | null {
  return profile === "drone" ? ["offload.stream", "atlas.ingest", "jobs.submit"] : null;
}

export const CAPTURE_CREDENTIALS: CredentialTarget = {
  plugin: CAPTURE_PLUGIN,
  lanesFor: captureLanes,
  installPath: "capture/node-credential",
};

export const RECONSTRUCT_CREDENTIALS: CredentialTarget = {
  plugin: RECONSTRUCT_PLUGIN,
  lanesFor: reconstructLanes,
  installPath: "reconstruct/node-credential",
};

/** Whether a listed credential carries exactly `lanes` (one extension's). */
export function sameLanes(a: readonly string[], b: readonly string[]): boolean {
  return a.length === b.length && a.every((l) => b.includes(l));
}
