/**
 * @module nodes/node-label
 * @description Names a node id or an offload `host:port` by the paired node
 * it belongs to, as the host's node list reports it. The raw id or address
 * is kept as the title. Pure.
 */

import { nodeToOffloadAddr } from "../vision/offload-target";

export interface NamedNode {
  deviceId: string;
  name: string;
  lanHost?: string | null;
}

export interface NodeLabel {
  label: string;
  title: string;
}

export function nodeLabelForId(id: string, nodes: readonly NamedNode[]): NodeLabel {
  const node = nodes.find((n) => n.deviceId === id);
  return { label: node?.name || id, title: id };
}

export function nodeLabelForAddr(addr: string, nodes: readonly NamedNode[]): NodeLabel {
  const node = nodes.find((n) => nodeToOffloadAddr({ lanHost: n.lanHost ?? null }) === addr);
  return { label: node?.name || addr, title: addr };
}
