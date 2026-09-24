/**
 * The sentinel fetch routes: a library handed a `.invalid` URL is answered from
 * the registered responder, every other request reaches the platform `fetch`
 * untouched, and the wrapper leaves `globalThis.fetch` exactly as it found it
 * once the last route is released.
 */

import { afterEach, beforeEach, describe, expect, it, vi, type Mock } from "vitest";

import { SENTINEL_ORIGIN, serveAtSentinel } from "../../../src/lib/net/sentinel-fetch";

type Fetch = typeof globalThis.fetch;

let original: Fetch;
let platform: Mock<Fetch>;

beforeEach(() => {
  original = globalThis.fetch;
  platform = vi.fn<Fetch>(() => Promise.resolve(new Response("platform")));
  globalThis.fetch = platform;
});

afterEach(() => {
  globalThis.fetch = original;
});

describe("serveAtSentinel", () => {
  it("answers its URL from the responder without touching the network", async () => {
    const route = serveAtSentinel("scene.ply", () => Promise.resolve(new Response("splat")));
    expect(route.url.startsWith(`${SENTINEL_ORIGIN}/`)).toBe(true);
    expect(route.url.endsWith("/scene.ply")).toBe(true);

    const res = await globalThis.fetch(route.url);
    expect(await res.text()).toBe("splat");
    expect(platform).not.toHaveBeenCalled();
    route.release();
  });

  it("matches a URL passed as a URL object or a Request, and forwards init", async () => {
    const respond = vi.fn((_init?: RequestInit) => Promise.resolve(new Response("ok")));
    const route = serveAtSentinel("a.bin", respond);
    await globalThis.fetch(new URL(route.url));
    await globalThis.fetch(new Request(route.url));
    const init = { method: "GET", headers: { Range: "bytes=0-1" } };
    await globalThis.fetch(route.url, init);
    expect(respond).toHaveBeenCalledTimes(3);
    expect(respond.mock.calls[2]?.[0]).toBe(init);
    expect(platform).not.toHaveBeenCalled();
    route.release();
  });

  it("passes every other request through to the platform fetch", async () => {
    const route = serveAtSentinel("a.bin", () => Promise.resolve(new Response("sentinel")));
    const init = { method: "POST" };
    const res = await globalThis.fetch("https://example.com/x", init);
    expect(await res.text()).toBe("platform");
    expect(platform).toHaveBeenCalledWith("https://example.com/x", init);
    // Another path on the sentinel origin is not a registered route either.
    await globalThis.fetch(`${SENTINEL_ORIGIN}/unregistered`);
    expect(platform).toHaveBeenCalledTimes(2);
    route.release();
  });

  it("gives two loads of one file distinct URLs unless the path is exact", () => {
    const a = serveAtSentinel("output.ply", () => Promise.resolve(new Response("a")));
    const b = serveAtSentinel("output.ply", () => Promise.resolve(new Response("b")));
    expect(a.url).not.toBe(b.url);
    const wasm = serveAtSentinel("rerun/re_viewer_bg.wasm", () => Promise.resolve(new Response("")), {
      exactPath: true,
    });
    expect(wasm.url).toBe(`${SENTINEL_ORIGIN}/rerun/re_viewer_bg.wasm`);
    a.release();
    b.release();
    wasm.release();
  });

  it("installs one wrapper and removes it only after the last release", async () => {
    const a = serveAtSentinel("a", () => Promise.resolve(new Response("a")));
    const wrapped = globalThis.fetch;
    expect(wrapped).not.toBe(platform);
    const b = serveAtSentinel("b", () => Promise.resolve(new Response("b")));
    expect(globalThis.fetch).toBe(wrapped);

    a.release();
    expect(globalThis.fetch).toBe(wrapped);
    // The released route now falls through to the platform.
    await globalThis.fetch(a.url);
    expect(platform).toHaveBeenCalledTimes(1);

    b.release();
    expect(globalThis.fetch).toBe(platform);
  });

  it("ignores a second release of the same route", () => {
    const a = serveAtSentinel("a", () => Promise.resolve(new Response("a")));
    const b = serveAtSentinel("b", () => Promise.resolve(new Response("b")));
    a.release();
    a.release();
    // b is still served, so the wrapper must still be installed.
    expect(globalThis.fetch).not.toBe(platform);
    b.release();
    expect(globalThis.fetch).toBe(platform);
  });

  it("leaves a wrapper added over it in place, still serving its routes", async () => {
    const route = serveAtSentinel("a", () => Promise.resolve(new Response("sentinel")));
    const ours = globalThis.fetch;
    const outer: Fetch = (input, init) => ours(input, init);
    globalThis.fetch = outer;

    route.release();
    // Unwinding would drop the outer wrapper; it stays, as a pass-through.
    expect(globalThis.fetch).toBe(outer);
    const res = await globalThis.fetch("https://example.com/y");
    expect(await res.text()).toBe("platform");

    // A new route is served through the chain without wrapping twice.
    const again = serveAtSentinel("b", () => Promise.resolve(new Response("again")));
    expect(globalThis.fetch).toBe(outer);
    expect(await (await globalThis.fetch(again.url)).text()).toBe("again");
    again.release();
  });
});
