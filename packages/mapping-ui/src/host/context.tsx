/**
 * The inline host API, handed to every component through React context. The
 * module is mounted once per page; everything below the page reads the host
 * through {@link useHost} instead of threading it through props.
 */

import { createContext, useContext, useMemo, type ReactNode } from "react";
import type { InlineAgentApi, InlineHostApi } from "@altnautica/plugin-sdk/inline";

/** The extension that runs the reconstruction node (its job API, outputs,
 * node credentials and live stream). */
export const RECONSTRUCT_PLUGIN = "com.altnautica.reconstruct";

const HostContext = createContext<InlineHostApi | null>(null);

export function HostProvider({ host, children }: { host: InlineHostApi; children: ReactNode }) {
  return <HostContext.Provider value={host}>{children}</HostContext.Provider>;
}

export function useHost(): InlineHostApi {
  const host = useContext(HostContext);
  if (!host) throw new Error("Mapping component rendered outside its host provider");
  return host;
}

/**
 * `plugin`'s HTTP server on `deviceId` (this extension's own by default; a
 * companion's under its `plugin.http.<plugin>` grant), through the mounted
 * node when it is that node and the node's own reach otherwise.
 */
export function agentOn(
  host: InlineHostApi,
  deviceId: string,
  plugin: string = host.plugin.id,
): InlineAgentApi {
  const own = plugin === host.plugin.id;
  if (deviceId === host.node.deviceId) return own ? host.agent : host.agentFor(plugin);
  return own ? host.nodes.agent(deviceId) : host.nodes.agent(deviceId, { plugin });
}

/** This extension's server on `deviceId`, or null when there is no id. */
export function useNodeAgent(deviceId: string | null | undefined): InlineAgentApi | null {
  const host = useHost();
  return useMemo(() => (deviceId ? agentOn(host, deviceId) : null), [host, deviceId]);
}

/** The reconstruction node's server on `deviceId`, or null when there is no
 * id. */
export function useReconstructAgent(deviceId: string | null | undefined): InlineAgentApi | null {
  const host = useHost();
  return useMemo(
    () => (deviceId ? agentOn(host, deviceId, RECONSTRUCT_PLUGIN) : null),
    [host, deviceId],
  );
}
