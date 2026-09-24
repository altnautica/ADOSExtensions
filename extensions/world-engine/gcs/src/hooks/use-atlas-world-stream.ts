/**
 * @module use-atlas-world-stream
 * @description Subscribes the paired compute node's per-device world-model
 * descriptor stream for one drone, and folds every frame into
 * `atlas-world-store`.
 *
 * The stream is the SHARED-DATA lane: the node publishes a splat / point-cloud /
 * mesh / occupancy descriptor per completed reconstruct generation at
 * `ws/atlas/<droneDeviceId>`, tagged with the drone that captured it.
 *
 * The node the stream is read from is passed in rather than resolved here:
 * `use-drone-world-model` already owns compute-node selection, and a second
 * resolver would be a second answer to the same question.
 *
 * Every stand-down reason is recorded as a distinct status so the surface can
 * say WHY there is no stream, which is never the same statement as "this drone
 * has no world model":
 *
 *  - **no-node** — nothing paired to stream from.
 *  - **blocked-origin** — Mission Control served over HTTPS cannot open a
 *    plain-`ws` LAN socket, and this lane has no server-side proxy, so the
 *    honest report is that the transport is unavailable on this origin.
 */

import { useEffect } from "react";

import { useNodeAgent } from "../host/context";
import { subscribeWorldStream, worldWsPath } from "../lib/atlas/world-stream";
import { useAtlasWorldStore } from "../stores/atlas-world-store";

/**
 * Mount the descriptor stream for `droneDeviceId` off the compute node
 * `computeNodeId`, writing into `atlas-world-store`. Inert (status recorded, no
 * socket) when either id is missing or on an HTTPS origin.
 *
 * @param droneDeviceId The capturing drone's device id — the stream path AND the
 *   store key, so one node serving several drones never cross-talks.
 * @param computeNodeId The reconstructor node's device id, from
 *   `useDroneWorldModel().computeNodeDeviceId`.
 */
export function useAtlasWorldStream(
  droneDeviceId: string | null | undefined,
  computeNodeId: string | null | undefined,
): void {
  const agent = useNodeAgent(computeNodeId);
  const drone = droneDeviceId ?? null;

  useEffect(() => {
    if (!drone) return;
    const { applyFrame, setStatus } = useAtlasWorldStore.getState();
    if (!agent) {
      setStatus(drone, "no-node");
      return;
    }
    // The host cannot open a plain-`ws` LAN socket from an HTTPS page (the
    // browser blocks it as mixed content). Saying so beats an endless
    // reconnect against a transport that cannot work on this origin.
    if (window.location.protocol === "https:") {
      setStatus(drone, "blocked-origin");
      return;
    }
    return subscribeWorldStream({
      open: () => agent.websocket(worldWsPath(drone)),
      onFrame: (frame) => applyFrame(drone, frame, Date.now()),
      onState: (state) => setStatus(drone, state),
    });
  }, [drone, agent]);

  // Stand the status down on unmount so a closed surface never reads as a live
  // stream. The generation itself is retained: the world model a node published
  // is still the newest thing known about this drone.
  useEffect(() => {
    if (!drone) return;
    return () => useAtlasWorldStore.getState().setStatus(drone, "idle");
  }, [drone]);
}
