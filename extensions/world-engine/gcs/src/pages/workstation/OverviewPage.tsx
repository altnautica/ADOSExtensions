/**
 * @module OverviewPage
 * @description A workstation's compute overview: its LAN posture, the GPU
 * card with its utilisation history, the compute-cluster status, and a live
 * glance at its jobs. The node's system vitals, logs and services are the
 * host's own overview.
 */

import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { ComputeClusterCard } from "./ComputeClusterCard";
import { GpuCard } from "./GpuCard";
import { GpuSparkline } from "./GpuSparkline";
import { JobsSummaryCard } from "./JobsSummaryCard";

const tAgent = translator("agent");

export function OverviewPage() {
  const { deviceId } = useHost().node;
  return (
    <div className="we:h-full we:space-y-4 we:overflow-y-auto we:p-4">
      {/* The node's job API and world-model artifacts are reachable by drones
          and other GCS on the same network, and that exposure is
          pairing-gated. */}
      <p className="we:text-xs we:text-text-secondary">{tAgent("computeLanNote")}</p>
      <div className="we:grid we:grid-cols-1 we:gap-4 we:md:grid-cols-2">
        <div className="we:space-y-3">
          <GpuCard deviceId={deviceId} />
          <GpuSparkline deviceId={deviceId} />
        </div>
        <div className="we:space-y-3">
          <ComputeClusterCard deviceId={deviceId} />
          <JobsSummaryCard deviceId={deviceId} />
        </div>
      </div>
    </div>
  );
}
