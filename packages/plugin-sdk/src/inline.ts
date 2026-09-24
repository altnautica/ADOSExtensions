/**
 * `@altnautica/plugin-sdk/inline`: the contract for a trusted inline GCS
 * module. A first-party signed extension that declares
 * `gcs.isolation: inline` ships an ES module the host imports into Mission
 * Control's own page (no iframe). Its default export, or a named export
 * `plugin`, is an {@link InlinePluginModule}; the host calls `mount` with the
 * element to render into and the {@link InlineHostApi}, and calls the
 * returned disposer when the page unmounts or the node changes.
 *
 * Build the module with `inlineSharedExternals()` from
 * `@altnautica/plugin-sdk/esbuild` so it uses the host's React.
 */

import type { NodeProfile } from "./manifest";
import type { PluginContext, PluginRecordsApi } from "./api";

export type {
  PluginRecord,
  PluginRecordListOptions,
  PluginRecordsApi,
} from "./api";

/** One paired node as `host.nodes.list()` reports it. */
export interface InlineNodeSummary {
  deviceId: string;
  name: string;
  profile: NodeProfile;
  /** Whether this browser can reach the node's agent right now. */
  reachable: boolean;
}

/**
 * What the host hands an inline module. Calls through `ctx` pass the same
 * capability checks as an iframe plugin's; a refusal rejects with an error
 * whose `code` names it (`refused`, `permission_denied`, `timeout`, ...).
 * Host capabilities that are unavailable here reject with an error whose
 * `code` is one of `websocket_unavailable_over_https_proxy`,
 * `no_node_agent`, `unknown_node`, `asset_unavailable`.
 */
export interface InlineHostApi {
  /** The plugin SDK context, served in-memory by the host. */
  ctx: PluginContext;
  plugin: { id: string; version: string; panelId: string; signerId: string };
  /** The node the module is mounted for, or null on a fleet-level slot. */
  node: { deviceId: string | null; profile: NodeProfile | null };
  agent: {
    /**
     * A request to the plugin's own HTTP server on the mounted node, through
     * the agent passthrough (`/api/plugins/{id}/x/<path>`). `path` may carry
     * a query string. Authenticated by the host.
     */
    fetch(path: string, init?: RequestInit): Promise<Response>;
    /**
     * A WebSocket to the same server. Rejects with
     * `websocket_unavailable_over_https_proxy` when Mission Control is served
     * over HTTPS (the node is plain HTTP on the local network).
     */
    websocket(path: string): Promise<WebSocket>;
  };
  /**
   * An object URL for a file under the plugin's `gcs/` directory (`path`
   * relative to it), typed by extension. Revoked when the module unmounts.
   */
  assetUrl(path: string): Promise<string>;
  /** The plugin's own cloud records (same as `ctx.records`). */
  records: PluginRecordsApi;
  nodes: {
    list(): InlineNodeSummary[];
    /** This plugin's config on another node, through that node's agent. */
    pluginConfig(deviceId: string): {
      get(): Promise<Record<string, unknown>>;
      set(key: string, value: unknown): Promise<void>;
    };
  };
  /**
   * Open one of this plugin's own pages: `agentPage` / `surface` name a
   * `gcs.contributes.agent_pages[].id` / `node_surfaces[].id`, on `nodeId`
   * (a device id) or the current node.
   */
  navigate(target: { nodeId?: string; surface?: string; agentPage?: string }): void;
}

/** The module an inline extension exports. */
export interface InlinePluginModule {
  mount(root: HTMLElement, host: InlineHostApi): (() => void) | Promise<() => void>;
}

/** The global the host publishes its React on before importing a module. */
export const INLINE_SHARED_GLOBAL = "__ADOS_INLINE_SHARED__";
