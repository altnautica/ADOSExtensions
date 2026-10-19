/**
 * Serves bytes to a library that insists on `fetch`ing a URL itself.
 *
 * The Rerun viewer fetches its wasm relative to a base URL, and the splat
 * renderer streams a level-of-detail file (and its chunk files) by URL with
 * `Range` requests. Neither can be handed an object URL, because the host
 * page's CSP does not let `fetch` read `blob:` URLs, and neither can send the
 * host's authentication. Instead each load registers a route on a reserved,
 * never-resolvable origin and a narrow `fetch` wrapper answers exactly those
 * URLs; every other request passes straight through to the platform `fetch`.
 * The wrapper is installed while at least one route is registered and removed
 * with the last one.
 */

/** A reserved `.invalid` origin (RFC 6761): nothing real ever answers it. */
export const SENTINEL_ORIGIN = "https://world-engine.invalid";

type Responder = (init?: RequestInit) => Promise<Response>;

/** One request under a tree route: the path below the route's base, the
 * request headers (a `Range` among them) and its abort signal. */
export interface SentinelTreeRequest {
  rel: string;
  headers: Headers;
  signal: AbortSignal | null;
}

type TreeResponder = (request: SentinelTreeRequest) => Promise<Response>;

const routes = new Map<string, Responder>();
const trees = new Map<string, TreeResponder>();
let platformFetch: typeof fetch | null = null;
let nextId = 0;

function requestUrl(input: RequestInfo | URL): string {
  if (typeof input === "string") return input;
  return input instanceof URL ? input.href : input.url;
}

function treeFor(url: string): { base: string; respond: TreeResponder } | null {
  for (const [base, respond] of trees) {
    if (url.startsWith(base)) return { base, respond };
  }
  return null;
}

const interceptor = (input: RequestInfo | URL, init?: RequestInit): Promise<Response> => {
  const url = requestUrl(input);
  const respond = routes.get(url);
  if (respond) return respond(init);
  const tree = treeFor(url);
  if (tree) {
    const req = input instanceof Request ? input : null;
    const headers = new Headers(init?.headers ?? req?.headers);
    const rel = (url.slice(tree.base.length).split(/[?#]/)[0] ?? "").replace(/^\/+/, "");
    return tree.respond({ rel, headers, signal: init?.signal ?? req?.signal ?? null });
  }
  if (!platformFetch) return Promise.reject(new TypeError("fetch is unavailable"));
  return platformFetch(input, init);
};

export interface SentinelRoute {
  /** The URL to hand the library. */
  url: string;
  /** Stop answering the URL (and restore `fetch` once no route is left). */
  release(): void;
}

function install(): void {
  // Install once. An interceptor still in the fetch chain (another wrapper
  // was added over it) keeps serving, so it is never wrapped a second time.
  if (platformFetch === null) {
    platformFetch = globalThis.fetch;
    globalThis.fetch = interceptor;
  }
}

function uninstallIfIdle(): void {
  // Only unwind our own wrapper: if something wrapped `fetch` after us, the
  // interceptor stays in its chain as a plain pass-through.
  if (routes.size === 0 && trees.size === 0 && globalThis.fetch === interceptor && platformFetch) {
    globalThis.fetch = platformFetch;
    platformFetch = null;
  }
}

/**
 * Answer requests for `<SENTINEL_ORIGIN>/<path>` with `respond`. Paths are
 * made unique unless `exactPath` is set, for a library that derives the URL
 * itself (a fixed name relative to a base).
 */
export function serveAtSentinel(
  path: string,
  respond: Responder,
  { exactPath = false }: { exactPath?: boolean } = {},
): SentinelRoute {
  const url = exactPath
    ? `${SENTINEL_ORIGIN}/${path}`
    : `${SENTINEL_ORIGIN}/${nextId++}/${path}`;
  install();
  routes.set(url, respond);
  return {
    url,
    release() {
      if (routes.get(url) !== respond) return;
      routes.delete(url);
      uninstallIfIdle();
    },
  };
}

/**
 * Answer every request under a unique `<SENTINEL_ORIGIN>/<id>/` base with
 * `respond`, for a library that derives sibling URLs from the one it was
 * given (chunk files next to a level-of-detail root). `url` is the base joined
 * with `entry`, the file the library is handed first.
 */
export function serveTreeAtSentinel(entry: string, respond: TreeResponder): SentinelRoute {
  const base = `${SENTINEL_ORIGIN}/t${nextId++}/`;
  install();
  trees.set(base, respond);
  return {
    url: `${base}${entry}`,
    release() {
      if (trees.get(base) !== respond) return;
      trees.delete(base);
      uninstallIfIdle();
    },
  };
}

/**
 * The bytes a `Range: bytes=a-b` request asked for, cut from a full `200`
 * answer by a server that ignored the header. A `206` or a request with no
 * range passes through unchanged.
 */
export async function honourRange(res: Response, range: string | null): Promise<Response> {
  if (!range || res.status !== 200) return res;
  const m = /^bytes=(\d+)-(\d*)$/.exec(range.trim());
  if (!m) return res;
  const body = new Uint8Array(await res.arrayBuffer());
  const start = Number(m[1]);
  const end = m[2] ? Math.min(Number(m[2]), body.length - 1) : body.length - 1;
  const slice = body.slice(start, end + 1);
  return new Response(slice, {
    status: 206,
    headers: { "Content-Range": `bytes ${start}-${end}/${body.length}` },
  });
}
