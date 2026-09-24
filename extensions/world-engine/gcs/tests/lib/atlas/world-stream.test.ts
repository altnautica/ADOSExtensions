/**
 * The descriptor-stream client: the per-device path, binary frame handoff,
 * reconnect, and teardown. The dialler is injected, so this exercises the real
 * state machine with no server.
 */

import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";

import {
  subscribeWorldStream,
  WORLD_WS_ROUTE,
  worldWsPath,
  type WorldStreamSocket,
  type WorldStreamState,
} from "../../../src/lib/atlas/world-stream";
import { SOCKET_RETRY_MS } from "../../../src/lib/net/reconnecting-socket";

/** A hand-driven stand-in for the browser socket. */
class FakeSocket implements WorldStreamSocket {
  binaryType = "";
  onopen: ((ev: Event) => void) | null = null;
  onmessage: ((ev: MessageEvent) => void) | null = null;
  onclose: ((ev: CloseEvent) => void) | null = null;
  onerror: ((ev: Event) => void) | null = null;
  closed = false;
  close() {
    this.closed = true;
  }
  open() {
    this.onopen?.(new Event("open"));
  }
  message(data: unknown) {
    this.onmessage?.(new MessageEvent("message", { data }));
  }
  drop() {
    this.onclose?.(new CloseEvent("close"));
  }
}

interface Harness {
  sockets: FakeSocket[];
  frames: Uint8Array[];
  states: WorldStreamState[];
  stop: () => void;
}

/** Subscribe with a dialler that resolves a fresh fake socket per dial, then
 * flush the dial's promise so the first socket is attached. */
async function subscribe(): Promise<Harness> {
  const sockets: FakeSocket[] = [];
  const frames: Uint8Array[] = [];
  const states: WorldStreamState[] = [];
  const stop = subscribeWorldStream({
    open: async () => {
      const s = new FakeSocket();
      sockets.push(s);
      return s;
    },
    onFrame: (f) => frames.push(f),
    onState: (s) => states.push(s),
  });
  await vi.advanceTimersByTimeAsync(0);
  return { sockets, frames, states, stop };
}

function nth(sockets: FakeSocket[], i: number): FakeSocket {
  const s = sockets[i];
  if (!s) throw new Error(`no socket ${i}`);
  return s;
}

describe("worldWsPath", () => {
  it("matches the agent's route shape, relative to the node's extension server", () => {
    expect(WORLD_WS_ROUTE).toBe("/ws/atlas/:device_id");
    expect(worldWsPath("drone-1")).toBe("ws/atlas/drone-1");
  });

  it("percent-encodes a device id so a path separator cannot escape the route", () => {
    expect(worldWsPath("dr one/7")).toBe("ws/atlas/dr%20one%2F7");
    expect(worldWsPath("../x")).toBe("ws/atlas/..%2Fx");
  });
});

describe("subscribeWorldStream", () => {
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => vi.useRealTimers());

  it("hands binary frames over as bytes and reports connect state", async () => {
    const h = await subscribe();
    const s = nth(h.sockets, 0);
    expect(h.states).toEqual(["connecting"]);
    expect(s.binaryType).toBe("arraybuffer");
    s.open();
    expect(h.states).toEqual(["connecting", "connected"]);

    s.message(new Uint8Array([1, 2, 3]).buffer);
    s.message(new Uint8Array([4]));
    expect(h.frames.map((f) => Array.from(f))).toEqual([[1, 2, 3], [4]]);

    h.stop();
    expect(s.closed).toBe(true);
  });

  it("ignores a text frame instead of counting it as a descriptor", async () => {
    const h = await subscribe();
    nth(h.sockets, 0).message("hello");
    expect(h.frames).toHaveLength(0);
    h.stop();
  });

  it("never reports its own teardown as a stream state", async () => {
    const h = await subscribe();
    nth(h.sockets, 0).open();
    h.stop();
    expect(h.states).toEqual(["connecting", "connected"]);
  });

  it("reconnects after a close at a fixed cadence that never grows", async () => {
    const h = await subscribe();
    nth(h.sockets, 0).drop();
    expect(h.states).toEqual(["connecting", "reconnecting"]);
    expect(h.sockets).toHaveLength(1);

    await vi.advanceTimersByTimeAsync(SOCKET_RETRY_MS - 1);
    expect(h.sockets).toHaveLength(1);
    await vi.advanceTimersByTimeAsync(1);
    expect(h.sockets).toHaveLength(2);

    // Keep failing; every retry lands after the same delay.
    for (let i = 0; i < 10; i++) {
      const before = h.sockets.length;
      nth(h.sockets, before - 1).drop();
      await vi.advanceTimersByTimeAsync(SOCKET_RETRY_MS);
      expect(h.sockets.length).toBe(before + 1);
    }
    h.stop();
  });

  it("does not shorten the delay when a socket opens and is closed at once", async () => {
    const h = await subscribe();
    nth(h.sockets, 0).drop();
    await vi.advanceTimersByTimeAsync(SOCKET_RETRY_MS);
    // A handler that accepts and closes immediately must not turn the
    // reconnect into a tight loop.
    nth(h.sockets, 1).open();
    nth(h.sockets, 1).drop();
    await vi.advanceTimersByTimeAsync(SOCKET_RETRY_MS - 1);
    expect(h.sockets).toHaveLength(2);
    await vi.advanceTimersByTimeAsync(1);
    expect(h.sockets).toHaveLength(3);
    h.stop();
  });

  it("stops reconnecting after teardown", async () => {
    const h = await subscribe();
    nth(h.sockets, 0).drop();
    h.stop();
    await vi.advanceTimersByTimeAsync(SOCKET_RETRY_MS * 5);
    expect(h.sockets).toHaveLength(1);
  });

  it("retries when the dial rejects rather than giving up", async () => {
    let calls = 0;
    const stop = subscribeWorldStream({
      open: async () => {
        calls += 1;
        throw new Error("refused");
      },
      onFrame: () => {},
      onState: () => {},
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(calls).toBe(1);
    await vi.advanceTimersByTimeAsync(SOCKET_RETRY_MS);
    expect(calls).toBe(2);
    stop();
  });

  it("closes a socket whose dial settles after teardown", async () => {
    let resolve: (s: FakeSocket) => void = () => {};
    const stop = subscribeWorldStream({
      open: () => new Promise<FakeSocket>((r) => (resolve = r)),
      onFrame: () => {},
      onState: () => {},
    });
    stop();
    const late = new FakeSocket();
    resolve(late);
    await vi.advanceTimersByTimeAsync(0);
    expect(late.closed).toBe(true);
  });
});
