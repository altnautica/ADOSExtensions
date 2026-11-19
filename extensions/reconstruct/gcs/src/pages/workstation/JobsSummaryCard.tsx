/**
 * @module JobsSummaryCard
 * @description Live glance of a compute node's reconstruction / offload jobs:
 * running / queued / failed counts, the top in-flight jobs with progress, and
 * a link to the full Compute page. Calm states when the node has no agent or
 * is unreachable, never a fabricated count.
 */

import type { ReactNode } from "react";
import { ChevronRight, Layers } from "lucide-react";

import { StatusDot, type StatusLevel } from "@altnautica/mapping-ui/components/ui/StatusDot";
import { useHost } from "@altnautica/mapping-ui/host/context";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";
import type { ComputeJob } from "@altnautica/mapping-ui/lib/agent/compute-client";

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
    <div className="rec:flex rec:items-center rec:gap-1.5">
      <StatusDot status={level} size="xs" />
      <span className="rec:font-mono rec:tabular-nums rec:text-text-primary">{value}</span>
      <span className="rec:text-text-tertiary">{label}</span>
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
    <div className="rec:space-y-2">
      <div className="rec:flex rec:flex-wrap rec:items-center rec:gap-x-3 rec:gap-y-1 rec:text-[11px]">
        <CountPill level="good" label={t("jobs.running")} value={running.length} />
        <CountPill level="idle" label={t("jobs.queued")} value={queued.length} />
        <CountPill level={failed ? "critical" : "idle"} label={t("jobs.failed")} value={failed} />
      </div>
      {top.length === 0 ? (
        <p className="rec:text-[10px] rec:text-text-tertiary">{t("jobs.none")}</p>
      ) : (
        <div className="rec:space-y-1">
          {top.map((j) => (
            <div key={j.id} className="rec:flex rec:items-center rec:gap-2">
              <StatusDot status={stateLevel(j.state)} size="xs" pulse={j.state === "running"} />
              <span className="rec:truncate rec:font-mono rec:text-[10px] rec:text-text-secondary">{j.kind}</span>
              {j.state === "running" && (
                <div className="rec:ml-auto rec:h-1 rec:w-16 rec:overflow-hidden rec:rounded-full rec:bg-bg-tertiary">
                  <div
                    className="rec:h-full rec:rounded-full rec:bg-accent-primary rec:transition-all"
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
    body = <p className="rec:text-[11px] rec:text-text-tertiary">{t("jobs.unavailable")}</p>;
  } else if (loading) {
    body = (
      <div className="rec:flex rec:items-center rec:justify-center rec:py-4">
        <div className="rec:h-4 rec:w-4 rec:animate-spin rec:rounded-full rec:border-2 rec:border-accent-primary rec:border-t-transparent" />
      </div>
    );
  } else if (unreachable) {
    body = <p className="rec:text-[11px] rec:text-text-tertiary">{t("jobs.awaiting")}</p>;
  } else {
    body = <JobGlance jobs={jobs} />;
  }

  return (
    <div className="rec:flex rec:h-full rec:flex-col rec:gap-2 rec:rounded-lg rec:border rec:border-border-default rec:bg-bg-secondary rec:p-3">
      <div className="rec:flex rec:items-center rec:justify-between">
        <div className="rec:flex rec:items-center rec:gap-1.5 rec:text-[11px] rec:uppercase rec:tracking-wide rec:text-text-tertiary">
          <Layers size={12} />
          <span>{t("jobs.title")}</span>
        </div>
        <button
          type="button"
          onClick={() => host.navigate({ surface: "compute" })}
          className="rec:group rec:flex rec:items-center rec:gap-0.5 rec:text-[10px] rec:text-text-tertiary rec:transition-colors rec:hover:text-accent-primary"
        >
          {t("jobs.viewAll")}
          <ChevronRight className="rec:h-3 rec:w-3 rec:transition-transform rec:group-hover:translate-x-0.5" />
        </button>
      </div>
      {body}
    </div>
  );
}
