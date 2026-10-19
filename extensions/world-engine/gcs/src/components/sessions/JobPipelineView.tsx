/**
 * @module sessions/JobPipelineView
 * @description One reconstruction job's pipeline: the stages with their
 * state, overall progress, GPU and ETA, the tool versions the node used, and
 * a failure (a missing tool names the tool to install).
 */

import { AlertTriangle, Check, Circle, Loader2, MinusCircle, X } from "lucide-react";

import { translator } from "../../host/i18n";
import type { ComputeJob } from "../../lib/agent/compute-client";
import { formatEta, pipelineFraction, stageRows, type StageState } from "../../lib/agent/job-pipeline";
import { cn } from "../../lib/utils";

const t = translator("pipeline");

const STAGE_ICON: Record<StageState, typeof Check> = {
  pending: Circle,
  running: Loader2,
  done: Check,
  failed: X,
  skipped: MinusCircle,
};

export function JobPipelineView({ job }: { job: ComputeJob }) {
  const p = job.pipeline;
  const err = job.errorInfo;
  const fraction = p ? pipelineFraction(p) : job.progress;
  const eta = p ? formatEta(p.etaS) : null;

  return (
    <div className="we:flex we:flex-col we:gap-1.5 we:text-[11px]" data-job={job.id}>
      <div className="we:flex we:items-center we:gap-2 we:text-text-secondary">
        <span className="we:font-mono">{job.kind}</span>
        {p?.preset && <span className="we:rounded we:bg-bg-tertiary we:px-1.5">{p.preset}</span>}
        <span className="we:text-text-tertiary">{job.state}</span>
        {fraction !== null && <span className="we:font-mono">{Math.round(fraction * 100)} %</span>}
        {eta && <span className="we:text-text-tertiary">{t("eta", { eta })}</span>}
      </div>
      {fraction !== null && (
        <div className="we:h-1 we:rounded we:bg-bg-tertiary" role="progressbar" aria-valuenow={Math.round(fraction * 100)} aria-valuemin={0} aria-valuemax={100}>
          <div className="we:h-1 we:rounded we:bg-accent-primary" style={{ width: `${fraction * 100}%` }} />
        </div>
      )}
      {p && (
        <ol className="we:flex we:flex-wrap we:gap-2" aria-label={t("stages")}>
          {stageRows(p).map((s) => {
            const Icon = STAGE_ICON[s.state];
            return (
              <li
                key={s.id}
                data-stage={s.id}
                data-state={s.state}
                className={cn(
                  "we:flex we:items-center we:gap-1",
                  s.state === "done" && "we:text-status-success",
                  s.state === "failed" && "we:text-status-error",
                  s.state === "running" && "we:text-accent-primary",
                  (s.state === "pending" || s.state === "skipped") && "we:text-text-tertiary",
                )}
              >
                <Icon size={11} className={s.state === "running" ? "we:animate-spin" : undefined} />
                {t(`stage.${s.id}`)}
                {s.state === "running" && s.progress !== null && ` ${Math.round(s.progress * 100)} %`}
              </li>
            );
          })}
        </ol>
      )}
      {p?.gpu && (
        <div className="we:text-text-tertiary">
          {t("gpu", {
            name: p.gpu.name ?? "–",
            util: p.gpu.utilPct !== null ? `${p.gpu.utilPct.toFixed(0)} %` : "–",
            vram: p.gpu.vramUsedMb !== null ? `${(p.gpu.vramUsedMb / 1024).toFixed(1)} GiB` : "–",
          })}
        </div>
      )}
      {p && p.tools.length > 0 && (
        <div className="we:font-mono we:text-text-tertiary">{p.tools.map((x) => `${x.name} ${x.version}`).join(" · ")}</div>
      )}
      {err && (
        <p role="alert" className="we:flex we:items-start we:gap-1 we:text-status-error">
          <AlertTriangle size={12} className="we:mt-px" />
          {err.code === "E_TOOL_MISSING" ? t("toolMissing", { tool: err.tool ?? "?" }) : `${err.code}: ${err.message}`}
        </p>
      )}
    </div>
  );
}
