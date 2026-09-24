/**
 * @module use-cloud-jobs
 * @description A drone's reconstruction jobs as the agent half recorded them
 * in the extension's cloud records (collection `jobs`, one record per job id,
 * `deviceId` = the capturing drone). The cloud fallback when no compute node
 * is reachable locally. Empty while signed out or offline.
 */

import { useEffect, useState } from "react";
import type { PluginRecord } from "@altnautica/plugin-sdk/inline";

import { useHost } from "../host/context";

/** The cloud records collection the agent half writes jobs to. */
export const JOBS_COLLECTION = "jobs";

/** How often the job records are re-read, in ms. */
export const CLOUD_JOBS_REFRESH_MS = 10_000;

/** One reconstruction job as recorded in the cloud. */
export interface CloudJob {
  /** The job id (the record key). */
  id: string;
  deviceId: string;
  computeNodeId: string | null;
  /** "splat" | "cloud" | "mesh" | "ortho". */
  kind: string;
  /** "queued" | "running" | "done" | "error" | "cancelled". */
  status: string;
  sessionId: string | null;
  outputUrl: string | null;
  /** Opaque producer metadata (gaussian count, steps, bounds, viewerHint, backend). */
  metadata: unknown;
  createdAt: number;
}

function str(v: unknown): string | null {
  return typeof v === "string" && v.length > 0 ? v : null;
}

/** Coerce one record into a job, or null when it is not one. */
export function coerceCloudJob(record: PluginRecord): CloudJob | null {
  const d = record.data;
  if (typeof d !== "object" || d === null || Array.isArray(d)) return null;
  const e = d as Record<string, unknown>;
  const deviceId = str(e.deviceId) ?? record.deviceId;
  if (!deviceId) return null;
  return {
    id: record.key,
    deviceId,
    computeNodeId: str(e.computeNodeId),
    kind: str(e.kind) ?? "",
    status: str(e.status) ?? "",
    sessionId: str(e.sessionId),
    outputUrl: str(e.outputUrl),
    metadata: e.metadata ?? null,
    createdAt: typeof e.createdAt === "number" ? e.createdAt : record.updatedAt,
  };
}

/** The drone's recorded jobs, newest first; empty until read or when records
 * are unavailable. */
export function useCloudJobs(deviceId: string | null): CloudJob[] {
  const host = useHost();
  const [jobs, setJobs] = useState<{ deviceId: string; list: CloudJob[] } | null>(null);

  useEffect(() => {
    if (!deviceId) return;
    let cancelled = false;
    const read = () =>
      host.records.list({ collection: JOBS_COLLECTION, deviceId, limit: 100 }).then(
        (records) => {
          if (cancelled) return;
          const list = records
            .flatMap((r) => {
              const job = coerceCloudJob(r);
              return job ? [job] : [];
            })
            .sort((a, b) => b.createdAt - a.createdAt);
          setJobs({ deviceId, list });
        },
        () => {
          // Signed out, offline or not granted: there is no cloud fallback.
          if (!cancelled) setJobs({ deviceId, list: [] });
        },
      );
    void read();
    const id = setInterval(() => void read(), CLOUD_JOBS_REFRESH_MS);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [host, deviceId]);

  return jobs && jobs.deviceId === deviceId ? jobs.list : [];
}
