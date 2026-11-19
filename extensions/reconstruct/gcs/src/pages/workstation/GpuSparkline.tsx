/**
 * @module GpuSparkline
 * @description Rolling GPU-utilisation sparkline for one compute node, drawn
 * as an inline SVG area over a fixed 0-100% scale from the node's GPU history.
 * The chart dims under a "paused" overlay once GPU reports stop arriving, and
 * renders nothing until two samples exist.
 */

import { useId, useMemo } from "react";

import { translator } from "../../host/i18n";
import { getFreshness, useNow } from "@altnautica/mapping-ui/lib/clock";
import { cn } from "@altnautica/mapping-ui/lib/utils";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../stores/compute-store";

const t = translator("node");
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
        "rec:border rec:border-border-default rec:rounded-lg rec:p-3 rec:bg-bg-secondary rec:transition-opacity",
        isStale && "rec:opacity-70",
      )}
    >
      <div className="rec:flex rec:items-center rec:justify-between rec:mb-2">
        <span className="rec:text-xs rec:text-text-secondary">{t("gpuHistory")}</span>
        <span className={cn("rec:text-xs rec:font-mono", isStale ? "rec:text-text-tertiary" : "rec:text-text-primary")}>
          {`${latest.toFixed(1)}%`}
        </span>
      </div>
      <div className="rec:relative rec:h-[60px] rec:w-full">
        <svg
          viewBox={`0 0 ${VIEW_W} ${VIEW_H}`}
          preserveAspectRatio="none"
          aria-hidden
          className={cn("rec:h-full rec:w-full", isStale ? "rec:text-text-tertiary" : "rec:text-accent-primary")}
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
          <div className="rec:absolute rec:inset-0 rec:flex rec:items-center rec:justify-center rec:pointer-events-none">
            <span className="rec:text-[10px] rec:uppercase rec:tracking-widest rec:text-text-tertiary rec:bg-bg-primary/70 rec:px-2 rec:py-0.5 rec:rounded">
              {tAgent("sparklinePaused")}
            </span>
          </div>
        )}
      </div>
    </div>
  );
}
