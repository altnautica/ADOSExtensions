/**
 * @module ComputePage
 * @description A workstation's Compute page: the job queue, the artifact
 * viewer over its finished jobs, and which paired drones and ground stations
 * may use this node's lanes, as segments of one pane.
 */

import { Boxes } from "lucide-react";

import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "../../hooks/use-compute-jobs";
import { CalmState } from "./CalmState";
import { DroneAccessPanel } from "./DroneAccessPanel";
import { ForgeOutputs } from "./ForgeOutputs";
import { JobsPanel } from "./JobsPanel";
import { SegmentedPane } from "./SegmentedPane";

const t = translator("atlas");

/** The reconstruction viewer over the node's finished jobs. */
function WorkstationViewer({ deviceId }: { deviceId: string | null }) {
  const { jobs, client } = useComputeJobs(deviceId);
  if (!client) return <CalmState icon={Boxes} message={t("forgeLocalOnly")} />;
  return <ForgeOutputs jobs={jobs} client={client} />;
}

export function ComputePage() {
  const { deviceId } = useHost().node;
  return (
    <SegmentedPane
      ariaLabel={t("computePaneLabel")}
      segments={[
        { id: "jobs", label: t("forgeJobs"), render: () => <JobsPanel deviceId={deviceId} /> },
        { id: "viewer", label: t("viewerGroupLabel"), render: () => <WorkstationViewer deviceId={deviceId} /> },
        { id: "access", label: t("droneAccess.segment"), render: () => <DroneAccessPanel deviceId={deviceId} /> },
      ]}
    />
  );
}
