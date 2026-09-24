/**
 * @module GpuSparkline
 * @description Rolling GPU-utilisation sparkline for one compute node, drawn
 * as an inline SVG area over a fixed 0-100% scale from the node's GPU history.
 * The chart dims under a "paused" overlay once GPU reports stop arriving, and
 * renders nothing until two samples exist.
 */

import { useId, useMemo } from "react";

import { translator } from "../../host/i18n";
import { getFreshness, useNow } from "../../lib/clock";
import { cn } from "../../lib/utils";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../stores/compute-store";

const t = translator("atlas");
const tAgent = translator("agent");

/** SVG user-space size; the chart stretches to its box. */
const VIEW_W = 100;
const VIEW_H = 60;

export function GpuSparkline({ deviceId }: { deviceId: string | null }) {
  const node = useComputeStore((s) => (deviceId ? s.nodes[deviceId] : undefined) ?? EMPTY_COMPUTE_NODE);
  const history = node.gpuHistory;
  const now = useNow();
  // useId output carries punctuation a `url(#…)` fragment does not accept.
  const gradientId = `we-gpu-${useId().replace(/[^\w-]/g, "")}`;
  const freshness = getFreshness(node.gpuUpdatedAt, now);
  const isStale = freshness.state === "stale" || freshness.state === "offline";

  // Recomputed only when the ring changes, not on every clock tick.
  const { line, area } = useMemo(() => {
    const last = history.length - 1;
    const pts = history.map((v, i) => {
      const x = last > 0 ? (i / last) * VIEW_W : 0;
      const y = VIEW_H - (Math.min(Math.max(v, 0), 100) / 100) * VIEW_H;
      return `${x.toFixed(2)},${y.toFixed(2)}`;
    });
    return {
      line: pts.join(" "),
      area: `M0,${VIEW_H} L${pts.join(" L")} L${VIEW_W},${VIEW_H} Z`,
    };
  }, [history]);

  const latest = history[history.length - 1];
  if (history.length < 2 || latest === undefined) return null;

  return (
    <div
      className={cn(
        "we:border we:border-border-default we:rounded-lg we:p-3 we:bg-bg-secondary we:transition-opacity",
        isStale && "we:opacity-70",
      )}
    >
      <div className="we:flex we:items-center we:justify-between we:mb-2">
        <span className="we:text-xs we:text-text-secondary">{t("gpuHistory")}</span>
        <span className={cn("we:text-xs we:font-mono", isStale ? "we:text-text-tertiary" : "we:text-text-primary")}>
          {`${latest.toFixed(1)}%`}
        </span>
      </div>
      <div className="we:relative we:h-[60px] we:w-full">
        <svg
          viewBox={`0 0 ${VIEW_W} ${VIEW_H}`}
          preserveAspectRatio="none"
          aria-hidden
          className={cn("we:h-full we:w-full", isStale ? "we:text-text-tertiary" : "we:text-accent-primary")}
        >
          <defs>
            <linearGradient id={gradientId} x1="0" y1="0" x2="0" y2="1">
              <stop offset="0%" stopColor="currentColor" stopOpacity={0.3} />
              <stop offset="100%" stopColor="currentColor" stopOpacity={0.05} />
            </linearGradient>
          </defs>
          <path d={area} fill={`url(#${gradientId})`} stroke="none" />
          <polyline
            points={line}
            fill="none"
            stroke="currentColor"
            strokeWidth={1.5}
            strokeLinejoin="round"
            vectorEffect="non-scaling-stroke"
          />
        </svg>
        {isStale && (
          <div className="we:absolute we:inset-0 we:flex we:items-center we:justify-center we:pointer-events-none">
            <span className="we:text-[10px] we:uppercase we:tracking-widest we:text-text-tertiary we:bg-bg-primary/70 we:px-2 we:py-0.5 we:rounded">
              {tAgent("sparklinePaused")}
            </span>
          </div>
        )}
      </div>
    </div>
  );
}
