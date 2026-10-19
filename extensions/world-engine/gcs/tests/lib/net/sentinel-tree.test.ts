/**
 * Tree routes on the sentinel origin: a level-of-detail root and its chunk
 * siblings are all answered by one responder that sees the path below the
 * route's base and the request's `Range`. Its own file: the wrapper's
 * install state is module-wide.
 */

import { afterEach, beforeEach, describe, expect, it, vi, type Mock } from "vitest";

import { serveTreeAtSentinel } from "../../../src/lib/net/sentinel-fetch";

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

describe("serveTreeAtSentinel", () => {
  it("answers the entry and every sibling under its base, with the request headers", async () => {
    const seen: { rel: string; range: string | null }[] = [];
    const route = serveTreeAtSentinel("scene-lod-0.rad", ({ rel, headers }) => {
      seen.push({ rel, range: headers.get("Range") });
      return Promise.resolve(new Response(rel));
    });
    expect(route.url.endsWith("/scene-lod-0.rad")).toBe(true);
    const sibling = new URL("scene-lod-3.rad", route.url).href;
    // The renderer fetches with a Request carrying a Range header.
    const req = new Request(route.url, { headers: { Range: "bytes=0-65535" } });
    expect(await (await globalThis.fetch(req)).text()).toBe("scene-lod-0.rad");
    expect(await (await globalThis.fetch(sibling)).text()).toBe("scene-lod-3.rad");
    expect(seen).toEqual([
      { rel: "scene-lod-0.rad", range: "bytes=0-65535" },
      { rel: "scene-lod-3.rad", range: null },
    ]);
    // Any other URL still reaches the platform.
    expect(await (await globalThis.fetch("https://example.com/a")).text()).toBe("platform");
    route.release();
    expect(globalThis.fetch).toBe(platform);
  });
});
