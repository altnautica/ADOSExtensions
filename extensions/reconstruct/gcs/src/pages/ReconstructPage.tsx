/**
 * The Reconstruct node surface (workstation and compute): a compact header
 * with the GPU, the cluster and a glance at the queue, then the Jobs,
 * Products and Editor views, which paired devices may use this node's lanes,
 * and the node's perception serving.
 */

import { useMemo } from "react";
import { Boxes } from "lucide-react";

import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";
import { SegmentedPane } from "@altnautica/mapping-ui/components/ui/SegmentedPane";
import { useHost } from "@altnautica/mapping-ui/host/context";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";

import { ChangeDetection } from "../components/sessions/ChangeDetection";
import { translator } from "../host/i18n";
import { OffloadPage } from "./offload/OffloadPage";
import { ComputeClusterCard } from "./workstation/ComputeClusterCard";
import { DroneAccessPanel } from "./workstation/DroneAccessPanel";
import { EditorView } from "./workstation/EditorView";
import { GpuCard } from "./workstation/GpuCard";
import { JobsPanel } from "./workstation/JobsPanel";
import { JobsSummaryCard } from "./workstation/JobsSummaryCard";
import { PipelinePanel } from "./workstation/PipelinePanel";
import { ProductsList } from "./workstation/ProductsList";

const t = translator("reconstruct");

/** The node's finished products, and comparing two of its sessions. */
function Products({ deviceId }: { deviceId: string | null }) {
  const { jobs, client } = useComputeJobs(deviceId);
  const sessionIds = useMemo(
    () => [...new Set(jobs.map((j) => j.sessionId).filter((s): s is string => Boolean(s)))],
    [jobs],
  );
  if (!client) return <CalmState icon={Boxes} message={t("localOnly")} />;
  return (
    <div className="rec:flex rec:h-full rec:flex-col rec:gap-3">
      <ProductsList jobs={jobs} client={client} />
      <div className="rec:border-t rec:border-border-default rec:p-3">
        <ChangeDetection sessionIds={sessionIds} nodeClient={client} />
      </div>
    </div>
  );
}

export function ReconstructPage() {
  const { deviceId } = useHost().node;
  return (
    <div className="rec:flex rec:h-full rec:min-h-0 rec:flex-col">
      <div className="rec:grid rec:shrink-0 rec:grid-cols-1 rec:gap-3 rec:p-3 rec:md:grid-cols-3">
        <GpuCard deviceId={deviceId} />
        <ComputeClusterCard deviceId={deviceId} />
        <JobsSummaryCard deviceId={deviceId} />
      </div>
      <SegmentedPane
        ariaLabel={t("viewsLabel")}
        segments={[
          {
            id: "jobs",
            label: t("jobs"),
            render: () => (
              <div className="rec:flex rec:flex-col rec:gap-3">
                <JobsPanel deviceId={deviceId} />
                <PipelinePanel deviceId={deviceId} />
              </div>
            ),
          },
          { id: "products", label: t("products"), render: () => <Products deviceId={deviceId} /> },
          { id: "editor", label: t("editor"), render: () => <EditorView deviceId={deviceId} /> },
          { id: "access", label: t("access"), render: () => <DroneAccessPanel deviceId={deviceId} /> },
          { id: "serving", label: t("serving"), render: () => <OffloadPage /> },
        ]}
      />
    </div>
  );
}
