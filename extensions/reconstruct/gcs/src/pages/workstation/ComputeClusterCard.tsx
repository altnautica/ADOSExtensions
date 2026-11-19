/**
 * @module ComputeClusterCard
 * @description Compute-cluster status for one node: its role (master / slave),
 * its job-queue depth and worker occupancy, the cluster's aggregate idle
 * capacity, live perception-offload sessions, and any registered slave nodes.
 * Renders an "awaiting heartbeat" state until the node's compute telemetry
 * lands, and dims a snapshot whose heartbeat has gone quiet.
 */

import type { ReactNode } from "react";
import { Boxes, Cpu, Layers, Radio } from "lucide-react";

import { translator } from "../../host/i18n";
import { useNow } from "@altnautica/mapping-ui/lib/clock";
import { cn, NO_DATA_GLYPH } from "@altnautica/mapping-ui/lib/utils";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../stores/compute-store";

const t = translator("node");

/** The compute heartbeat is ~1 Hz, so a snapshot older than this is stale: an
 * operator must not dispatch jobs to a node whose heartbeat has gone quiet. */
const STALE_MS = 15_000;

/** Compact "5s" / "2m" heartbeat-age label. */
function ago(ms: number): string {
  const s = Math.max(0, Math.round(ms / 1000));
  return s < 60 ? `${s}s` : `${Math.round(s / 60)}m`;
}

function num(v: number | null): string {
  return v === null ? NO_DATA_GLYPH : String(v);
}

function roleBadgeClass(role: string): string {
  if (role === "master") return "rec:bg-accent-primary/15 rec:text-accent-primary";
  if (role === "slave") return "rec:bg-text-primary/[0.06] rec:text-text-secondary";
  return "rec:bg-text-primary/[0.04] rec:text-text-tertiary";
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="rec:rounded rec:bg-text-primary/[0.02] rec:px-2 rec:py-1.5 rec:text-center">
      <div className="rec:text-sm rec:font-mono rec:text-text-primary rec:tabular-nums">{value}</div>
      <div className="rec:text-[9px] rec:uppercase rec:tracking-wide rec:text-text-tertiary">{label}</div>
    </div>
  );
}

function Header({ children }: { children?: ReactNode }) {
  return (
    <div className="rec:flex rec:items-center rec:justify-between">
      <div className="rec:flex rec:items-center rec:gap-1.5">
        <Boxes className="rec:w-3.5 rec:h-3.5 rec:text-text-tertiary" />
        <span className="rec:text-xs rec:font-medium rec:text-text-secondary">{t("computeCluster")}</span>
      </div>
      {children}
    </div>
  );
}

export function ComputeClusterCard({ deviceId, className }: { deviceId: string | null; className?: string }) {
  const cluster = useComputeStore((s) => (deviceId ? s.nodes[deviceId] : undefined) ?? EMPTY_COMPUTE_NODE).cluster;
  const now = useNow();

  if (cluster.role === null) {
    return (
      <div className={cn("rec:border rec:border-border-default rec:rounded-lg rec:p-4 rec:space-y-3", className)}>
        <Header />
        <div className="rec:text-[10px] rec:text-text-tertiary rec:text-center rec:py-3">{t("awaitingHeartbeat")}</div>
      </div>
    );
  }

  const badgeLabel =
    cluster.role === "master" ? t("master") : cluster.role === "slave" ? t("slave") : cluster.role;
  const age = cluster.updatedAt === null ? null : now - cluster.updatedAt;
  const isStale = age === null || age > STALE_MS;

  return (
    <div className={cn("rec:border rec:border-border-default rec:rounded-lg rec:p-4 rec:space-y-3", className)}>
      <Header>
        <div className="rec:flex rec:items-center rec:gap-1.5">
          {isStale && (
            <span
              className="rec:text-[10px] rec:font-medium rec:px-1.5 rec:py-0.5 rec:rounded rec:bg-status-warning/15 rec:text-status-warning"
              title={age === null ? "no heartbeat" : `last heartbeat ${ago(age)} ago`}
            >
              {t("stale")}
            </span>
          )}
          <span
            className={cn(
              "rec:text-[10px] rec:font-medium rec:px-1.5 rec:py-0.5 rec:rounded",
              roleBadgeClass(cluster.role),
            )}
          >
            {badgeLabel}
          </span>
        </div>
      </Header>

      <div className={cn("rec:grid rec:grid-cols-3 rec:gap-2", isStale && "rec:opacity-50")}>
        <Stat label={t("queue")} value={num(cluster.queueDepth)} />
        <Stat label={t("active")} value={num(cluster.activeJobs)} />
        <Stat label={t("idle")} value={num(cluster.workersIdle)} />
      </div>

      <div
        className={cn(
          "rec:flex rec:items-center rec:justify-between rec:border-t rec:border-border-default rec:pt-2",
          isStale && "rec:opacity-50",
        )}
      >
        <span className="rec:text-[10px] rec:text-text-secondary rec:flex rec:items-center rec:gap-1">
          <Layers className="rec:w-3 rec:h-3 rec:text-text-tertiary" />
          {t("clusterIdleWorkers")}
        </span>
        <span className="rec:text-[10px] rec:font-mono rec:text-text-primary rec:tabular-nums">
          {num(cluster.aggregateWorkersIdle)}
        </span>
      </div>

      {cluster.activeSessions !== null && (
        <div className={cn("rec:flex rec:items-center rec:justify-between", isStale && "rec:opacity-50")}>
          <span className="rec:text-[10px] rec:text-text-secondary rec:flex rec:items-center rec:gap-1">
            <Radio className="rec:w-3 rec:h-3 rec:text-text-tertiary" />
            {t("servingSessions")}
          </span>
          <span className="rec:text-[10px] rec:font-mono rec:text-text-primary rec:tabular-nums">
            {num(cluster.activeSessions)}
          </span>
        </div>
      )}

      {cluster.masterId && (
        <div className="rec:flex rec:items-center rec:justify-between">
          <span className="rec:text-[10px] rec:text-text-tertiary">{t("master")}</span>
          <span
            className="rec:text-[10px] rec:font-mono rec:text-text-secondary rec:truncate rec:max-w-[60%]"
            title={cluster.masterId}
          >
            {cluster.masterId}
          </span>
        </div>
      )}

      {cluster.slaves.length > 0 && (
        <div className="rec:pt-2 rec:border-t rec:border-border-default rec:space-y-1.5">
          <span className="rec:text-[10px] rec:text-text-tertiary">
            {t("slaves")} ({cluster.slaves.length})
          </span>
          {cluster.slaves.map((s) => (
            <div
              key={s.nodeId}
              className="rec:flex rec:items-center rec:gap-2 rec:px-2 rec:py-1 rec:rounded rec:bg-text-primary/[0.02]"
            >
              <Cpu size={10} className="rec:text-text-tertiary rec:flex-shrink-0" />
              <span
                className="rec:text-[10px] rec:font-mono rec:text-text-secondary rec:truncate"
                title={s.accelerators.length > 0 ? s.accelerators.join(", ") : s.nodeId}
              >
                {s.nodeId}
              </span>
              <span className="rec:text-[10px] rec:font-mono rec:text-text-tertiary rec:ml-auto rec:flex-shrink-0 rec:tabular-nums">
                {num(s.workersIdle)} idle · {num(s.queueDepth)} q
              </span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
