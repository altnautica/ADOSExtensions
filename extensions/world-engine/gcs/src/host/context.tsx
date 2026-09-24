/**
 * The inline host API, handed to every component through React context. The
 * module is mounted once per page; everything below the page reads the host
 * through {@link useHost} instead of threading it through props.
 */

import { createContext, useContext, useMemo, type ReactNode } from "react";
import type { InlineAgentApi, InlineHostApi } from "@altnautica/plugin-sdk/inline";

const HostContext = createContext<InlineHostApi | null>(null);

export function HostProvider({ host, children }: { host: InlineHostApi; children: ReactNode }) {
  return <HostContext.Provider value={host}>{children}</HostContext.Provider>;
}

export function useHost(): InlineHostApi {
  const host = useContext(HostContext);
  if (!host) throw new Error("World Engine component rendered outside its host provider");
  return host;
}

/**
 * This extension's HTTP server on `deviceId` (the mounted node when it is that
 * node, another paired node's otherwise), or null when there is no id.
 */
export function useNodeAgent(deviceId: string | null | undefined): InlineAgentApi | null {
  const host = useHost();
  return useMemo(() => {
    if (!deviceId) return null;
    return deviceId === host.node.deviceId ? host.agent : host.nodes.agent(deviceId);
  }, [host, deviceId]);
}
