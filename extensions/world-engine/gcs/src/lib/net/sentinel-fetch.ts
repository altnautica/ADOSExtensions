/**
 * Serves bytes to a library that insists on `fetch`ing a URL itself.
 *
 * The Rerun viewer fetches its wasm relative to a base URL, and the splat
 * viewer fetches its scene from a URL. Neither can be handed an object URL,
 * because the host page's CSP does not let `fetch` read `blob:` URLs, and
 * neither can send the host's authentication. Instead each load registers a
 * route on a reserved, never-resolvable origin and a narrow `fetch` wrapper
 * answers exactly those URLs; every other request passes straight through to
 * the platform `fetch`. The wrapper is installed while at least one route is
 * registered and removed with the last one.
 */

/** A reserved `.invalid` origin (RFC 6761): nothing real ever answers it. */
export const SENTINEL_ORIGIN = "https://world-engine.invalid";

type Responder = (init?: RequestInit) => Promise<Response>;

const routes = new Map<string, Responder>();
let platformFetch: typeof fetch | null = null;
let nextId = 0;

function requestUrl(input: RequestInfo | URL): string {
  if (typeof input === "string") return input;
  return input instanceof URL ? input.href : input.url;
}

const interceptor = (input: RequestInfo | URL, init?: RequestInit): Promise<Response> => {
  const respond = routes.get(requestUrl(input));
  if (respond) return respond(init);
  if (!platformFetch) return Promise.reject(new TypeError("fetch is unavailable"));
  return platformFetch(input, init);
};

export interface SentinelRoute {
  /** The URL to hand the library. */
  url: string;
  /** Stop answering the URL (and restore `fetch` once no route is left). */
  release(): void;
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
  // Install once. An interceptor still in the fetch chain (another wrapper
  // was added over it) keeps serving, so it is never wrapped a second time.
  if (platformFetch === null) {
    platformFetch = globalThis.fetch;
    globalThis.fetch = interceptor;
  }
  routes.set(url, respond);
  return {
    url,
    release() {
      if (routes.get(url) !== respond) return;
      routes.delete(url);
      // Only unwind our own wrapper: if something wrapped `fetch` after us, the
      // interceptor stays in its chain as a plain pass-through.
      if (routes.size === 0 && globalThis.fetch === interceptor && platformFetch) {
        globalThis.fetch = platformFetch;
        platformFetch = null;
      }
    },
  };
}
