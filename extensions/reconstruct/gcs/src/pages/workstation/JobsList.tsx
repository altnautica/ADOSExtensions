/**
 * @module JobsList
 * @description A compute node's reconstruction / offload job rows with state
 * badges, progress, a cancel affordance for in-flight jobs and a re-run for
 * terminal ones.
 *
 * Both write paths consume their result: a cancel the engine refuses puts the
 * button back and says so, and a re-run reports whether it was queued.
 */

import { useEffect, useState } from "react";
import { Layers, RotateCcw, X } from "lucide-react";

import { useToast } from "@altnautica/mapping-ui/components/ui/Toast";
import { translator } from "../../host/i18n";
import type { ComputeAgentClient, ComputeJob } from "@altnautica/mapping-ui/lib/agent/compute-client";
import { qualityForSteps } from "@altnautica/mapping-ui/lib/reconstruction-quality";
import { cn, NO_DATA_GLYPH } from "@altnautica/mapping-ui/lib/utils";

const t = translator("node");

/** i18n key per known job state (an unknown state renders raw). */
export const JOB_STATE_KEYS: Record<string, string> = {
  queued: "jobQueued",
  running: "jobRunning",
  completed: "jobCompleted",
  failed: "jobFailed",
  cancelled: "jobCancelled",
};

function jobStateClass(state: string): string {
  switch (state) {
    case "running":
      return "rec:bg-accent-primary/15 rec:text-accent-primary";
    case "completed":
      return "rec:bg-status-success/15 rec:text-status-success";
    case "failed":
      return "rec:bg-status-error/15 rec:text-status-error";
    default:
      return "rec:bg-bg-tertiary rec:text-text-tertiary";
  }
}

/** Compact age of an epoch-ms stamp ("12s" / "4m" / "2h"). */
function ago(ms: number): string {
  if (ms <= 0) return NO_DATA_GLYPH;
  const s = Math.max(0, Math.round((Date.now() - ms) / 1000));
  if (s < 60) return `${s}s`;
  if (s < 3600) return `${Math.round(s / 60)}m`;
  return `${Math.round(s / 3600)}h`;
}

function JobRow({
  job,
  onCancel,
  onRetry,
}: {
  job: ComputeJob;
  onCancel: ((id: string) => void) | null;
  onRetry: ((job: ComputeJob) => void) | null;
}) {
  const stateKey = JOB_STATE_KEYS[job.state];
  const cancellable = job.state === "queued" || job.state === "running";
  const retryable = job.state === "failed" || job.state === "cancelled";

  return (
    <div className="rec:flex rec:items-center rec:gap-2 rec:px-2 rec:py-1.5 rec:rounded rec:bg-bg-tertiary">
      <Layers size={12} className="rec:text-text-tertiary rec:flex-shrink-0" />
      <div className="rec:min-w-0 rec:flex-1">
        <div className="rec:flex rec:items-center rec:gap-2">
          <span className="rec:text-[11px] rec:font-mono rec:text-text-secondary rec:truncate">{job.kind}</span>
          {job.kind === "reconstruct" && job.steps !== null && (
            <span
              className="rec:text-[10px] rec:font-medium rec:px-1.5 rec:py-0.5 rec:rounded rec:bg-bg-tertiary rec:text-text-tertiary rec:flex-shrink-0"
              title={`${job.steps.toLocaleString()} steps`}
            >
              {t(qualityForSteps(job.steps).labelKey)}
            </span>
          )}
          <span className="rec:text-[10px] rec:font-mono rec:text-text-tertiary rec:truncate" title={job.id}>
            {job.id}
          </span>
        </div>
        {job.state === "running" && (
          <div className="rec:h-1 rec:mt-1 rec:bg-bg-tertiary rec:rounded-full rec:overflow-hidden">
            <div
              className="rec:h-full rec:rounded-full rec:bg-accent-primary rec:transition-all"
              style={{ width: `${Math.min(job.progress * 100, 100)}%` }}
            />
          </div>
        )}
        {job.error && (
          <p className="rec:text-[10px] rec:text-status-error rec:truncate" title={job.error}>
            {job.error}
          </p>
        )}
      </div>
      <span className="rec:text-[10px] rec:font-mono rec:text-text-tertiary rec:flex-shrink-0 rec:tabular-nums">
        {ago(job.updatedMs || job.createdMs)}
      </span>
      <span
        className={cn(
          "rec:text-[10px] rec:font-medium rec:px-1.5 rec:py-0.5 rec:rounded rec:flex-shrink-0",
          jobStateClass(job.state),
        )}
      >
        {stateKey ? t(stateKey) : job.state}
      </span>
      {cancellable && onCancel && (
        <button
          type="button"
          onClick={() => onCancel(job.id)}
          title={t("forgeCancel")}
          aria-label={t("forgeCancel")}
          className="rec:text-text-tertiary rec:hover:text-status-error rec:transition-colors rec:flex-shrink-0"
        >
          <X size={12} />
        </button>
      )}
      {retryable && onRetry && (
        <button
          type="button"
          onClick={() => onRetry(job)}
          title={t("forgeRetry")}
          aria-label={t("forgeRetry")}
          className="rec:text-text-tertiary rec:hover:text-accent-primary rec:transition-colors rec:flex-shrink-0"
        >
          <RotateCcw size={12} />
        </button>
      )}
    </div>
  );
}

export function JobsList({ jobs, client }: { jobs: readonly ComputeJob[]; client: ComputeAgentClient | null }) {
  const { toast } = useToast();
  // Ids asked to cancel, so the button hides immediately (the next poll
  // reflects the engine's terminal state).
  const [cancelling, setCancelling] = useState<ReadonlySet<string>>(new Set());

  // A node switch mints a new client; an optimistic id from the previous node
  // must not hide the cancel control on an unrelated job with the same id.
  useEffect(() => {
    setCancelling(new Set());
  }, [client]);

  const onCancel = client
    ? (id: string) => {
        setCancelling((prev) => new Set(prev).add(id));
        void (async () => {
          if (await client.cancelJob(id)) return;
          // Refused (already finished, unreachable, 5xx): restore the control.
          setCancelling((prev) => {
            const next = new Set(prev);
            next.delete(id);
            return next;
          });
          toast(t("forgeCancelFailed"), "error");
        })();
      }
    : null;

  const onRetry = client
    ? (job: ComputeJob) => {
        void (async () => {
          // The original params verbatim: without device_id or generation the
          // re-run is unattributed and never published.
          const result = await client.submitJob({
            kind: job.kind,
            datasetId: job.datasetId ?? undefined,
            params: job.params,
          });
          toast(
            result === null ? t("forgeRetryFailed") : t("forgeRetryQueued"),
            result === null ? "error" : "success",
          );
        })();
      }
    : null;

  if (jobs.length === 0) {
    return <div className="rec:text-[11px] rec:text-text-tertiary rec:text-center rec:py-8">{t("forgeNoJobs")}</div>;
  }

  return (
    <div className="rec:space-y-1.5 rec:p-1">
      {jobs.map((job) => (
        <JobRow key={job.id} job={job} onCancel={cancelling.has(job.id) ? null : onCancel} onRetry={onRetry} />
      ))}
    </div>
  );
}
