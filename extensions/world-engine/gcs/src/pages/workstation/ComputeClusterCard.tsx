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
import { useNow } from "../../lib/clock";
import { cn, NO_DATA_GLYPH } from "../../lib/utils";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../stores/compute-store";

const t = translator("atlas");

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
  if (role === "master") return "we:bg-accent-primary/15 we:text-accent-primary";
  if (role === "slave") return "we:bg-text-primary/[0.06] we:text-text-secondary";
  return "we:bg-text-primary/[0.04] we:text-text-tertiary";
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="we:rounded we:bg-text-primary/[0.02] we:px-2 we:py-1.5 we:text-center">
      <div className="we:text-sm we:font-mono we:text-text-primary we:tabular-nums">{value}</div>
      <div className="we:text-[9px] we:uppercase we:tracking-wide we:text-text-tertiary">{label}</div>
    </div>
  );
}

function Header({ children }: { children?: ReactNode }) {
  return (
    <div className="we:flex we:items-center we:justify-between">
      <div className="we:flex we:items-center we:gap-1.5">
        <Boxes className="we:w-3.5 we:h-3.5 we:text-text-tertiary" />
        <span className="we:text-xs we:font-medium we:text-text-secondary">{t("computeCluster")}</span>
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
      <div className={cn("we:border we:border-border-default we:rounded-lg we:p-4 we:space-y-3", className)}>
        <Header />
        <div className="we:text-[10px] we:text-text-tertiary we:text-center we:py-3">{t("awaitingHeartbeat")}</div>
      </div>
    );
  }

  const badgeLabel =
    cluster.role === "master" ? t("master") : cluster.role === "slave" ? t("slave") : cluster.role;
  const age = cluster.updatedAt === null ? null : now - cluster.updatedAt;
  const isStale = age === null || age > STALE_MS;

  return (
    <div className={cn("we:border we:border-border-default we:rounded-lg we:p-4 we:space-y-3", className)}>
      <Header>
        <div className="we:flex we:items-center we:gap-1.5">
          {isStale && (
            <span
              className="we:text-[10px] we:font-medium we:px-1.5 we:py-0.5 we:rounded we:bg-status-warning/15 we:text-status-warning"
              title={age === null ? "no heartbeat" : `last heartbeat ${ago(age)} ago`}
            >
              {t("stale")}
            </span>
          )}
          <span
            className={cn(
              "we:text-[10px] we:font-medium we:px-1.5 we:py-0.5 we:rounded",
              roleBadgeClass(cluster.role),
            )}
          >
            {badgeLabel}
          </span>
        </div>
      </Header>

      <div className={cn("we:grid we:grid-cols-3 we:gap-2", isStale && "we:opacity-50")}>
        <Stat label={t("queue")} value={num(cluster.queueDepth)} />
        <Stat label={t("active")} value={num(cluster.activeJobs)} />
        <Stat label={t("idle")} value={num(cluster.workersIdle)} />
      </div>

      <div
        className={cn(
          "we:flex we:items-center we:justify-between we:border-t we:border-border-default we:pt-2",
          isStale && "we:opacity-50",
        )}
      >
        <span className="we:text-[10px] we:text-text-secondary we:flex we:items-center we:gap-1">
          <Layers className="we:w-3 we:h-3 we:text-text-tertiary" />
          {t("clusterIdleWorkers")}
        </span>
        <span className="we:text-[10px] we:font-mono we:text-text-primary we:tabular-nums">
          {num(cluster.aggregateWorkersIdle)}
        </span>
      </div>

      {cluster.activeSessions !== null && (
        <div className={cn("we:flex we:items-center we:justify-between", isStale && "we:opacity-50")}>
          <span className="we:text-[10px] we:text-text-secondary we:flex we:items-center we:gap-1">
            <Radio className="we:w-3 we:h-3 we:text-text-tertiary" />
            {t("servingSessions")}
          </span>
          <span className="we:text-[10px] we:font-mono we:text-text-primary we:tabular-nums">
            {num(cluster.activeSessions)}
          </span>
        </div>
      )}

      {cluster.masterId && (
        <div className="we:flex we:items-center we:justify-between">
          <span className="we:text-[10px] we:text-text-tertiary">{t("master")}</span>
          <span
            className="we:text-[10px] we:font-mono we:text-text-secondary we:truncate we:max-w-[60%]"
            title={cluster.masterId}
          >
            {cluster.masterId}
          </span>
        </div>
      )}

      {cluster.slaves.length > 0 && (
        <div className="we:pt-2 we:border-t we:border-border-default we:space-y-1.5">
          <span className="we:text-[10px] we:text-text-tertiary">
            {t("slaves")} ({cluster.slaves.length})
          </span>
          {cluster.slaves.map((s) => (
            <div
              key={s.nodeId}
              className="we:flex we:items-center we:gap-2 we:px-2 we:py-1 we:rounded we:bg-text-primary/[0.02]"
            >
              <Cpu size={10} className="we:text-text-tertiary we:flex-shrink-0" />
              <span
                className="we:text-[10px] we:font-mono we:text-text-secondary we:truncate"
                title={s.accelerators.length > 0 ? s.accelerators.join(", ") : s.nodeId}
              >
                {s.nodeId}
              </span>
              <span className="we:text-[10px] we:font-mono we:text-text-tertiary we:ml-auto we:flex-shrink-0 we:tabular-nums">
                {num(s.workersIdle)} idle · {num(s.queueDepth)} q
              </span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
