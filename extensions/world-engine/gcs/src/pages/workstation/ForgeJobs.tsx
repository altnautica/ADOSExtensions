/**
 * @module ForgeJobs
 * @description A compute node's reconstruction / offload job rows with state
 * badges, progress, a cancel affordance for in-flight jobs and a re-run for
 * terminal ones.
 *
 * Both write paths consume their result: a cancel the engine refuses puts the
 * button back and says so, and a re-run reports whether it was queued.
 */

import { useEffect, useState } from "react";
import { Layers, RotateCcw, X } from "lucide-react";

import { useToast } from "../../components/ui/Toast";
import { translator } from "../../host/i18n";
import type { ComputeAgentClient, ComputeJob } from "../../lib/agent/compute-client";
import { qualityForSteps } from "../../lib/atlas/reconstruction-quality";
import { cn, NO_DATA_GLYPH } from "../../lib/utils";

const t = translator("atlas");

/** atlas i18n key per known job state (an unknown state renders raw). */
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
      return "we:bg-accent-primary/15 we:text-accent-primary";
    case "completed":
      return "we:bg-status-success/15 we:text-status-success";
    case "failed":
      return "we:bg-status-error/15 we:text-status-error";
    default:
      return "we:bg-bg-tertiary we:text-text-tertiary";
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
    <div className="we:flex we:items-center we:gap-2 we:px-2 we:py-1.5 we:rounded we:bg-bg-tertiary">
      <Layers size={12} className="we:text-text-tertiary we:flex-shrink-0" />
      <div className="we:min-w-0 we:flex-1">
        <div className="we:flex we:items-center we:gap-2">
          <span className="we:text-[11px] we:font-mono we:text-text-secondary we:truncate">{job.kind}</span>
          {job.kind === "reconstruct" && job.steps !== null && (
            <span
              className="we:text-[10px] we:font-medium we:px-1.5 we:py-0.5 we:rounded we:bg-bg-tertiary we:text-text-tertiary we:flex-shrink-0"
              title={`${job.steps.toLocaleString()} steps`}
            >
              {t(qualityForSteps(job.steps).labelKey)}
            </span>
          )}
          <span className="we:text-[10px] we:font-mono we:text-text-tertiary we:truncate" title={job.id}>
            {job.id}
          </span>
        </div>
        {job.state === "running" && (
          <div className="we:h-1 we:mt-1 we:bg-bg-tertiary we:rounded-full we:overflow-hidden">
            <div
              className="we:h-full we:rounded-full we:bg-accent-primary we:transition-all"
              style={{ width: `${Math.min(job.progress * 100, 100)}%` }}
            />
          </div>
        )}
        {job.error && (
          <p className="we:text-[10px] we:text-status-error we:truncate" title={job.error}>
            {job.error}
          </p>
        )}
      </div>
      <span className="we:text-[10px] we:font-mono we:text-text-tertiary we:flex-shrink-0 we:tabular-nums">
        {ago(job.updatedMs || job.createdMs)}
      </span>
      <span
        className={cn(
          "we:text-[10px] we:font-medium we:px-1.5 we:py-0.5 we:rounded we:flex-shrink-0",
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
          className="we:text-text-tertiary we:hover:text-status-error we:transition-colors we:flex-shrink-0"
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
          className="we:text-text-tertiary we:hover:text-accent-primary we:transition-colors we:flex-shrink-0"
        >
          <RotateCcw size={12} />
        </button>
      )}
    </div>
  );
}

export function ForgeJobs({ jobs, client }: { jobs: readonly ComputeJob[]; client: ComputeAgentClient | null }) {
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
    return <div className="we:text-[11px] we:text-text-tertiary we:text-center we:py-8">{t("forgeNoJobs")}</div>;
  }

  return (
    <div className="we:space-y-1.5 we:p-1">
      {jobs.map((job) => (
        <JobRow key={job.id} job={job} onCancel={cancelling.has(job.id) ? null : onCancel} onRetry={onRetry} />
      ))}
    </div>
  );
}
