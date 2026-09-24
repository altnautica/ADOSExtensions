/**
 * Where a viewer reads a reconstruction artifact's bytes from.
 *
 * A compute node stamps each artifact's `uri` with a host derived from its own
 * OS hostname (an mDNS `.local` name the browser cannot resolve, and that
 * drifts between runs). The PATH under `/artifacts/` is stable, so an artifact
 * on a paired compute node is read through that node's extension server
 * (`artifacts/<relpath>`), which the host authenticates and routes (LAN,
 * relay or the HTTPS proxy). A URL that names no artifact path (an external
 * store) is fetched as-is.
 *
 * The bytes are always read with `fetch` from here, never handed to a library
 * as an object URL: the host page's CSP does not let `fetch` read `blob:` URLs.
 */

import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

export interface ArtifactSource {
  /** Stable identity: two sources with one key read the same bytes. */
  key: string;
  /** The artifact's file name; its extension names the format. */
  name: string;
  fetch(signal: AbortSignal): Promise<Response>;
}

/** Pull the stable `artifacts/<relpath>` segment out of a stored artifact URL,
 * ignoring the engine's (drifting, unresolvable) host. */
export function artifactRelPath(rawUri: string): string | null {
  const fromPath = (p: string): string | null => {
    const i = p.indexOf("artifacts/");
    return i >= 0 ? p.slice(i) : null;
  };
  try {
    const u = new URL(rawUri);
    return fromPath(u.pathname.replace(/^\/+/, ""));
  } catch {
    // Not an absolute URL — treat the input as a bare path.
    return fromPath(rawUri.replace(/^\/+/, ""));
  }
}

function fileName(path: string): string {
  const clean = path.split(/[?#]/)[0] ?? path;
  return clean.slice(clean.lastIndexOf("/") + 1);
}

/**
 * The source for an artifact `uri` a compute node reported: through
 * `node.agent` when the uri carries an artifact path, otherwise the uri itself.
 * Null for a uri that is neither (a `mock://` placeholder).
 */
export function artifactSource(
  uri: string,
  node: { deviceId: string; agent: InlineAgentApi } | null,
): ArtifactSource | null {
  const rel = artifactRelPath(uri);
  if (rel && node) {
    return {
      key: `${node.deviceId}:${rel}`,
      name: fileName(rel),
      fetch: (signal) => node.agent.fetch(rel, { signal }),
    };
  }
  if (!/^https?:\/\//i.test(uri)) return null;
  return { key: uri, name: fileName(uri), fetch: (signal) => fetch(uri, { signal }) };
}
