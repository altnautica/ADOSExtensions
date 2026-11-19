/**
 * @module GpuCard
 * @description A workstation's GPU: identity (name, cores, unified memory,
 * Metal family) plus a live utilisation bar from the node's compute telemetry.
 * Renders "awaiting GPU telemetry" before the first report, one sentence when
 * the node reports no hardware GPU (reconstruction on the CPU), drops the bar when
 * the node reports no utilisation (never a fabricated 0%), and greys it once
 * reports stop arriving.
 */

import { Activity, Cpu } from "lucide-react";

import { translator } from "../../host/i18n";
import { getFreshness, useNow } from "@altnautica/mapping-ui/lib/clock";
import { cn, NO_DATA_GLYPH } from "@altnautica/mapping-ui/lib/utils";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../stores/compute-store";

const t = translator("node");

/** Threshold fill for a utilisation bar: accent below 70%, warning to 90%,
 * error above; a stale reading greys out. */
function barColor(percent: number, stale: boolean): string {
  if (stale) return "rec:bg-text-tertiary/60";
  if (percent >= 90) return "rec:bg-status-error";
  if (percent >= 70) return "rec:bg-status-warning";
  return "rec:bg-accent-primary";
}

function UtilizationBar({ percent, stale, staleLabel }: { percent: number; stale: boolean; staleLabel: string }) {
  return (
    <div className="rec:space-y-1">
      <div className="rec:flex rec:items-center rec:justify-between">
        <div className="rec:flex rec:items-center rec:gap-1.5">
          <Activity size={12} className="rec:text-text-tertiary" />
          <span className="rec:text-xs rec:text-text-secondary">{t("gpu")}</span>
        </div>
        <span
          className={cn("rec:text-xs rec:font-mono", stale ? "rec:text-text-tertiary" : "rec:text-text-primary")}
          title={stale ? `Last reading ${staleLabel}` : undefined}
        >
          {`${percent.toFixed(1)}%`}
        </span>
      </div>
      <div className="rec:h-1.5 rec:bg-bg-tertiary rec:rounded-full rec:overflow-hidden">
        <div
          className={cn("rec:h-full rec:rounded-full rec:transition-all", barColor(percent, stale))}
          style={{ width: `${Math.min(percent, 100)}%` }}
        />
      </div>
      <p className="rec:text-[10px] rec:text-text-tertiary">{t("utilization", { pct: percent.toFixed(1) })}</p>
    </div>
  );
}

export function GpuCard({ deviceId, className }: { deviceId: string | null; className?: string }) {
  const node = useComputeStore((s) => (deviceId ? s.nodes[deviceId] : undefined) ?? EMPTY_COMPUTE_NODE);
  const now = useNow();
  const { gpu } = node;
  const util = gpu?.utilizationPct ?? null;
  const freshness = getFreshness(node.gpuUpdatedAt, now);
  const stale = freshness.state === "stale" || freshness.state === "offline";

  return (
    <div className={cn("rec:border rec:border-border-default rec:rounded-lg rec:p-4 rec:space-y-3", className)}>
      <div className="rec:flex rec:items-center rec:justify-between">
        <div className="rec:flex rec:items-center rec:gap-1.5">
          <Cpu className="rec:w-3.5 rec:h-3.5 rec:text-text-tertiary" />
          <span className="rec:text-xs rec:font-medium rec:text-text-secondary">{t("gpu")}</span>
        </div>
        {gpu?.metal && (
          <span
            className="rec:text-[10px] rec:font-medium rec:px-1.5 rec:py-0.5 rec:rounded rec:bg-accent-primary/15 rec:text-accent-primary"
            title={gpu.metal}
          >
            {t("metal")}
          </span>
        )}
      </div>

      {gpu === null ? (
        <div className="rec:text-[10px] rec:text-text-tertiary rec:text-center rec:py-3">{t("gpuAwaiting")}</div>
      ) : gpu.present === false ? (
        <p className="rec:text-xs rec:text-text-secondary">{t("gpuNone")}</p>
      ) : (
        <>
          <div className="rec:flex rec:items-center rec:justify-between rec:gap-2">
            <span className="rec:text-[11px] rec:font-mono rec:text-text-primary rec:truncate">
              {gpu.name ?? NO_DATA_GLYPH}
            </span>
            {gpu.cores != null && (
              <span className="rec:text-[10px] rec:font-mono rec:text-text-tertiary rec:flex-shrink-0">
                {t("gpuCores", { cores: gpu.cores })}
              </span>
            )}
          </div>

          {util != null ? (
            <UtilizationBar percent={util} stale={stale} staleLabel={freshness.label} />
          ) : (
            <div className="rec:flex rec:items-center rec:gap-1.5 rec:text-[10px] rec:text-text-tertiary">
              <Activity size={10} className="rec:flex-shrink-0" />
              <span>{t("gpuAwaiting")}</span>
            </div>
          )}

          {gpu.unifiedMemoryMb != null && Number.isFinite(gpu.unifiedMemoryMb) && (
            <div className="rec:flex rec:items-center rec:justify-between rec:border-t rec:border-border-default rec:pt-2">
              <span className="rec:text-[10px] rec:text-text-secondary">{t("unifiedMemory")}</span>
              <span className="rec:text-[10px] rec:font-mono rec:text-text-primary">
                {`${Math.round(gpu.unifiedMemoryMb / 1024)} GB`}
              </span>
            </div>
          )}
        </>
      )}
    </div>
  );
}
