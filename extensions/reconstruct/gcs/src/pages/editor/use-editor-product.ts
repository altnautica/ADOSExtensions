/**
 * @module editor/use-editor-product
 * @description One product open in the editor: which version is current, its
 * provenance, the decoded cloud, whether this tab may write (one writer per
 * product across tabs), running an edit job to completion, and the leave
 * prompt while operations are queued.
 */

import { useCallback, useEffect, useState } from "react";

import { decodeLazEnu } from "@altnautica/mapping-ui/components/model/viewers/cloud-laz";
import type { ComputeAgentClient } from "@altnautica/mapping-ui/lib/agent/compute-client";

import {
  outRel,
  readLatest,
  readProvenance,
  submitEdit,
  unedited,
  versionsOf,
  type EditOp,
  type EditsFile,
} from "./editor-api";
import { buildEditorCloud, type EditorCloud } from "./editor-cloud";

/** The reconstruction's own cloud, the first version of every edit chain. */
export const BASE_ARTIFACT = "out/cloud.laz";
/** How often a running edit job is polled. */
const POLL_MS = 1000;

function sleep(ms: number): Promise<void> {
  const { promise, resolve } = Promise.withResolvers<void>();
  setTimeout(resolve, ms);
  return promise;
}

export type LoadState =
  | { kind: "loading" }
  | { kind: "missing" }
  | { kind: "failed" }
  | { kind: "ready"; cloud: EditorCloud; edits: EditsFile };

/**
 * Whether another tab already edits this product: a tab announces itself on a
 * channel keyed by node and product, and the first one there answers every
 * later one, which opens read-only.
 */
export function useSingleWriter(key: string): boolean {
  const [otherWriter, setOtherWriter] = useState(false);
  useEffect(() => {
    setOtherWriter(false);
    if (typeof BroadcastChannel === "undefined") return;
    const ch = new BroadcastChannel(`reconstruct-editor:${key}`);
    let writer = true;
    ch.onmessage = (e: MessageEvent<string>) => {
      if (e.data === "hello" && writer) ch.postMessage("here");
      if (e.data === "here") {
        writer = false;
        setOtherWriter(true);
      }
    };
    ch.postMessage("hello");
    return () => ch.close();
  }, [key]);
  return otherWriter;
}

/** The browser asks once before the tab closes while `dirty`. */
export function useLeavePrompt(dirty: boolean): void {
  useEffect(() => {
    if (!dirty) return;
    const ask = (e: BeforeUnloadEvent) => {
      e.preventDefault();
    };
    window.addEventListener("beforeunload", ask);
    return () => window.removeEventListener("beforeunload", ask);
  }, [dirty]);
}

export function useEditorProduct(client: ComputeAgentClient, job: string) {
  const [version, setVersion] = useState<string | null>(null);
  const [state, setState] = useState<LoadState>({ kind: "loading" });
  const [running, setRunning] = useState<string | null>(null);
  /** Every version the newest edit's provenance names, oldest first. */
  const [chain, setChain] = useState<string[]>([]);

  // Open the newest version the product's edits wrote, else the base.
  useEffect(() => {
    let cancelled = false;
    setVersion(null);
    void readLatest(client, job).then((latest) => {
      if (cancelled) return;
      setVersion(latest?.product ?? BASE_ARTIFACT);
      setChain(latest ? versionsOf(latest) : []);
    });
    return () => {
      cancelled = true;
    };
  }, [client, job]);

  useEffect(() => {
    if (!version) return;
    let cancelled = false;
    const abort = new AbortController();
    setState({ kind: "loading" });
    void (async () => {
      try {
        const res = await client.fileSource(job, outRel(version)).fetch(abort.signal);
        if (res.status === 404) {
          if (!cancelled) setState({ kind: "missing" });
          return;
        }
        if (!res.ok) throw new Error(String(res.status));
        const [buffer, prov] = await Promise.all([res.arrayBuffer(), readProvenance(client, job, version)]);
        const decoded = await decodeLazEnu(buffer);
        const points = decoded.positions.length / 3;
        const edits = prov ?? unedited(version, points);
        const cloud = buildEditorCloud(decoded.positions, decoded.colors, edits.layers);
        if (!cancelled) setState({ kind: "ready", cloud, edits });
      } catch {
        if (!cancelled) setState({ kind: "failed" });
      }
    })();
    return () => {
      cancelled = true;
      abort.abort();
    };
  }, [client, job, version]);

  /**
   * Run `ops` on the open version as one edit job and open what it wrote.
   * Resolves to null on success, else the sentence to show.
   */
  const run = useCallback(
    async (session: string, ops: EditOp[]): Promise<string | null> => {
      if (!version) return null;
      const submitted = await submitEdit(client, session, job, version, ops);
      if (!submitted.ok) return submitted.message;
      setRunning(submitted.jobId);
      try {
        for (;;) {
          await sleep(POLL_MS);
          const j = await client.getJob(submitted.jobId);
          if (!j) continue;
          if (j.state === "completed") break;
          if (j.state === "failed" || j.state === "cancelled") return j.errorInfo?.message ?? j.error ?? j.state;
        }
      } finally {
        setRunning(null);
      }
      if (ops.some((o) => o.op !== "export")) {
        const latest = await readLatest(client, job);
        if (latest) {
          setVersion(latest.product);
          setChain(versionsOf(latest));
        }
      }
      return null;
    },
    [client, job, version],
  );

  return { version, setVersion, state, run, running, chain };
}
