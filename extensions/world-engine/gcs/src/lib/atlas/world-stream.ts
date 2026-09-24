/**
 * @module lib/atlas/world-stream
 * @description Client for the compute node's per-device world-model descriptor
 * stream.
 *
 * The node fans out world-model descriptors on a broadcast channel and serves
 * them over one WebSocket per device (`/ws/atlas/<device_id>`), beside the
 * job API rather than through the bounded MAVLink queue. Each descriptor is
 * tagged with the drone it belongs to and the handler filters to its own
 * device, so a multi-device node never cross-talks one drone's world into
 * another's view.
 *
 * Frames are binary msgpack `AtlasEvent`s. This module moves BYTES only: the
 * envelope check and the descriptor decode belong to the store, so a refusal is
 * counted in one place.
 *
 * A lagged subscriber is skipped by the publisher rather than blocking the
 * trainer, which is safe here in a way it would not be for a delta lane: each
 * descriptor is a complete statement about one generation, so a skipped frame
 * costs the consumer that generation and never desynchronises it.
 */

import { openReconnectingSocket, type ReconnectingSocketLike } from "../net/reconnecting-socket";

/** The route the node serves, one path per device (the agent's own constant). */
export const WORLD_WS_ROUTE = "/ws/atlas/:device_id";

/** The stream path for a device, relative to the node's extension server. */
export function worldWsPath(deviceId: string): string {
  return `ws/atlas/${encodeURIComponent(deviceId)}`;
}

/** The subset of `WebSocket` this client drives, so a test can inject one. */
export interface WorldStreamSocket extends ReconnectingSocketLike {
  binaryType: string;
}

export type WorldStreamState = "connecting" | "connected" | "reconnecting";

export interface WorldStreamOptions {
  /** Dial one socket to the stream (the host authenticates it). */
  open: () => Promise<WorldStreamSocket>;
  /** One decoded-nothing frame: raw envelope bytes. */
  onFrame: (frame: Uint8Array) => void;
  onState: (state: WorldStreamState) => void;
}

/** Open the descriptor stream, reconnecting until the returned unsubscribe is
 * called. */
export function subscribeWorldStream(opts: WorldStreamOptions): () => void {
  return openReconnectingSocket({
    open: () =>
      opts.open().then((ws) => {
        ws.binaryType = "arraybuffer";
        return ws;
      }),
    onMessage: (data) => {
      // Descriptors are binary. A text frame is off-contract; ignoring it keeps
      // a chatty proxy from being counted as a malformed descriptor.
      if (data instanceof ArrayBuffer) {
        opts.onFrame(new Uint8Array(data));
      } else if (data instanceof Uint8Array) {
        opts.onFrame(data);
      }
    },
    // Teardown is the caller's own act, not a stream state it tracks.
    onState: (state) => {
      if (state !== "closed") opts.onState(state);
    },
  });
}
