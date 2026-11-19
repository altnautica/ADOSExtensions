/**
 * @module use-paired-nodes
 * @description The operator's paired nodes as the host reports them,
 * re-read on an interval (the host list is a snapshot, not a subscription).
 * The returned array keeps its identity until the list actually changes, so
 * it is safe as an effect or memo dependency.
 */

import { useEffect, useState } from "react";
import type { InlineNodeSummary } from "@altnautica/plugin-sdk/inline";

import { useHost } from "../host/context";

/** How often the paired-node list is re-read, in ms. */
export const PAIRED_NODES_REFRESH_MS = 5000;

export function usePairedNodes(): readonly InlineNodeSummary[] {
  const host = useHost();
  const [nodes, setNodes] = useState<readonly InlineNodeSummary[]>(() => host.nodes.list());
  useEffect(() => {
    const read = () => {
      const next = host.nodes.list();
      setNodes((prev) => (JSON.stringify(prev) === JSON.stringify(next) ? prev : next));
    };
    read();
    const id = setInterval(read, PAIRED_NODES_REFRESH_MS);
    return () => clearInterval(id);
  }, [host]);
  return nodes;
}
