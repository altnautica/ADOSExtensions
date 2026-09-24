/**
 * @module JobsSummaryCard
 * @description Live glance of a compute node's reconstruction / offload jobs:
 * running / queued / failed counts, the top in-flight jobs with progress, and
 * a link to the full Compute page. Calm states when the node has no agent or
 * is unreachable, never a fabricated count.
 */

import type { ReactNode } from "react";
import { ChevronRight, Layers } from "lucide-react";

import { StatusDot, type StatusLevel } from "../../components/ui/StatusDot";
import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "../../hooks/use-compute-jobs";
import type { ComputeJob } from "../../lib/agent/compute-client";

const t = translator("nodeConsole");

function stateLevel(state: string): StatusLevel {
  switch (state) {
    case "running":
    case "completed":
      return "good";
    case "failed":
      return "critical";
    default:
      return "idle";
  }
}

function CountPill({ level, label, value }: { level: StatusLevel; label: string; value: number }) {
  return (
    <div className="we:flex we:items-center we:gap-1.5">
      <StatusDot status={level} size="xs" />
      <span className="we:font-mono we:tabular-nums we:text-text-primary">{value}</span>
      <span className="we:text-text-tertiary">{label}</span>
    </div>
  );
}

function JobGlance({ jobs }: { jobs: readonly ComputeJob[] }) {
  const running = jobs.filter((j) => j.state === "running");
  const queued = jobs.filter((j) => j.state === "queued");
  const failed = jobs.filter((j) => j.state === "failed").length;
  // Top in-flight jobs: running first, then queued, capped at three.
  const top = [...running, ...queued].slice(0, 3);
  return (
    <div className="we:space-y-2">
      <div className="we:flex we:flex-wrap we:items-center we:gap-x-3 we:gap-y-1 we:text-[11px]">
        <CountPill level="good" label={t("jobs.running")} value={running.length} />
        <CountPill level="idle" label={t("jobs.queued")} value={queued.length} />
        <CountPill level={failed ? "critical" : "idle"} label={t("jobs.failed")} value={failed} />
      </div>
      {top.length === 0 ? (
        <p className="we:text-[10px] we:text-text-tertiary">{t("jobs.none")}</p>
      ) : (
        <div className="we:space-y-1">
          {top.map((j) => (
            <div key={j.id} className="we:flex we:items-center we:gap-2">
              <StatusDot status={stateLevel(j.state)} size="xs" pulse={j.state === "running"} />
              <span className="we:truncate we:font-mono we:text-[10px] we:text-text-secondary">{j.kind}</span>
              {j.state === "running" && (
                <div className="we:ml-auto we:h-1 we:w-16 we:overflow-hidden we:rounded-full we:bg-bg-tertiary">
                  <div
                    className="we:h-full we:rounded-full we:bg-accent-primary we:transition-all"
                    style={{ width: `${Math.min(j.progress * 100, 100)}%` }}
                  />
                </div>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

export function JobsSummaryCard({ deviceId }: { deviceId: string | null }) {
  const host = useHost();
  const { jobs, loading, unreachable, client } = useComputeJobs(deviceId);

  let body: ReactNode;
  if (!client) {
    body = <p className="we:text-[11px] we:text-text-tertiary">{t("jobs.unavailable")}</p>;
  } else if (loading) {
    body = (
      <div className="we:flex we:items-center we:justify-center we:py-4">
        <div className="we:h-4 we:w-4 we:animate-spin we:rounded-full we:border-2 we:border-accent-primary we:border-t-transparent" />
      </div>
    );
  } else if (unreachable) {
    body = <p className="we:text-[11px] we:text-text-tertiary">{t("jobs.awaiting")}</p>;
  } else {
    body = <JobGlance jobs={jobs} />;
  }

  return (
    <div className="we:flex we:h-full we:flex-col we:gap-2 we:rounded-lg we:border we:border-border-default we:bg-bg-secondary we:p-3">
      <div className="we:flex we:items-center we:justify-between">
        <div className="we:flex we:items-center we:gap-1.5 we:text-[11px] we:uppercase we:tracking-wide we:text-text-tertiary">
          <Layers size={12} />
          <span>{t("jobs.title")}</span>
        </div>
        <button
          type="button"
          onClick={() => host.navigate({ surface: "compute" })}
          className="we:group we:flex we:items-center we:gap-0.5 we:text-[10px] we:text-text-tertiary we:transition-colors we:hover:text-accent-primary"
        >
          {t("jobs.viewAll")}
          <ChevronRight className="we:h-3 we:w-3 we:transition-transform we:group-hover:translate-x-0.5" />
        </button>
      </div>
      {body}
    </div>
  );
}
