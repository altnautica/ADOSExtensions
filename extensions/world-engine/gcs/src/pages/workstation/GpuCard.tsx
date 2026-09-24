/**
 * @module GpuCard
 * @description A workstation's GPU: identity (name, cores, unified memory,
 * Metal family) plus a live utilisation bar from the node's compute telemetry.
 * Renders "awaiting GPU telemetry" before the first report, drops the bar when
 * the node reports no utilisation (never a fabricated 0%), and greys it once
 * reports stop arriving.
 */

import { Activity, Cpu } from "lucide-react";

import { translator } from "../../host/i18n";
import { getFreshness, useNow } from "../../lib/clock";
import { cn, NO_DATA_GLYPH } from "../../lib/utils";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../stores/compute-store";

const t = translator("atlas");

/** Threshold fill for a utilisation bar: accent below 70%, warning to 90%,
 * error above; a stale reading greys out. */
function barColor(percent: number, stale: boolean): string {
  if (stale) return "we:bg-text-tertiary/60";
  if (percent >= 90) return "we:bg-status-error";
  if (percent >= 70) return "we:bg-status-warning";
  return "we:bg-accent-primary";
}

function UtilizationBar({ percent, stale, staleLabel }: { percent: number; stale: boolean; staleLabel: string }) {
  return (
    <div className="we:space-y-1">
      <div className="we:flex we:items-center we:justify-between">
        <div className="we:flex we:items-center we:gap-1.5">
          <Activity size={12} className="we:text-text-tertiary" />
          <span className="we:text-xs we:text-text-secondary">{t("gpu")}</span>
        </div>
        <span
          className={cn("we:text-xs we:font-mono", stale ? "we:text-text-tertiary" : "we:text-text-primary")}
          title={stale ? `Last reading ${staleLabel}` : undefined}
        >
          {`${percent.toFixed(1)}%`}
        </span>
      </div>
      <div className="we:h-1.5 we:bg-bg-tertiary we:rounded-full we:overflow-hidden">
        <div
          className={cn("we:h-full we:rounded-full we:transition-all", barColor(percent, stale))}
          style={{ width: `${Math.min(percent, 100)}%` }}
        />
      </div>
      <p className="we:text-[10px] we:text-text-tertiary">{t("utilization", { pct: percent.toFixed(1) })}</p>
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
    <div className={cn("we:border we:border-border-default we:rounded-lg we:p-4 we:space-y-3", className)}>
      <div className="we:flex we:items-center we:justify-between">
        <div className="we:flex we:items-center we:gap-1.5">
          <Cpu className="we:w-3.5 we:h-3.5 we:text-text-tertiary" />
          <span className="we:text-xs we:font-medium we:text-text-secondary">{t("gpu")}</span>
        </div>
        {gpu?.metal && (
          <span
            className="we:text-[10px] we:font-medium we:px-1.5 we:py-0.5 we:rounded we:bg-accent-primary/15 we:text-accent-primary"
            title={gpu.metal}
          >
            {t("metal")}
          </span>
        )}
      </div>

      {gpu === null ? (
        <div className="we:text-[10px] we:text-text-tertiary we:text-center we:py-3">{t("gpuAwaiting")}</div>
      ) : (
        <>
          <div className="we:flex we:items-center we:justify-between we:gap-2">
            <span className="we:text-[11px] we:font-mono we:text-text-primary we:truncate">
              {gpu.name ?? NO_DATA_GLYPH}
            </span>
            {gpu.cores != null && (
              <span className="we:text-[10px] we:font-mono we:text-text-tertiary we:flex-shrink-0">
                {t("gpuCores", { cores: gpu.cores })}
              </span>
            )}
          </div>

          {util != null ? (
            <UtilizationBar percent={util} stale={stale} staleLabel={freshness.label} />
          ) : (
            <div className="we:flex we:items-center we:gap-1.5 we:text-[10px] we:text-text-tertiary">
              <Activity size={10} className="we:flex-shrink-0" />
              <span>{t("gpuAwaiting")}</span>
            </div>
          )}

          {gpu.unifiedMemoryMb != null && Number.isFinite(gpu.unifiedMemoryMb) && (
            <div className="we:flex we:items-center we:justify-between we:border-t we:border-border-default we:pt-2">
              <span className="we:text-[10px] we:text-text-secondary">{t("unifiedMemory")}</span>
              <span className="we:text-[10px] we:font-mono we:text-text-primary">
                {`${Math.round(gpu.unifiedMemoryMb / 1024)} GB`}
              </span>
            </div>
          )}
        </>
      )}
    </div>
  );
}
