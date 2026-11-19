/**
 * @module PipelinePanel
 * @description A compute node's staged reconstruction jobs (reconstruct,
 * change, calibrate): each job's stages, GPU, ETA, tool versions and failure,
 * newest first. A job refused for a missing tool names the tool to provision.
 */

import { Workflow } from "lucide-react";

import { JobPipelineView } from "@altnautica/mapping-ui/components/sessions/JobPipelineView";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";

const t = translator("pipeline");

/** How many finished staged jobs stay listed under the active ones. */
const RECENT_FINISHED = 8;

export function PipelinePanel({ deviceId }: { deviceId: string | null }) {
  const { jobs, loading, unreachable, client } = useComputeJobs(deviceId);
  if (!client) return <CalmState icon={Workflow} message={t("localOnly")} />;
  if (unreachable) return <CalmState icon={Workflow} message={t("unreachable")} />;
  const staged = jobs.filter((j) => j.pipeline !== null).sort((a, b) => b.createdMs - a.createdMs);
  const active = staged.filter((j) => j.state === "queued" || j.state === "running");
  const finished = staged.filter((j) => j.state !== "queued" && j.state !== "running").slice(0, RECENT_FINISHED);
  if (!loading && staged.length === 0) return <CalmState icon={Workflow} message={t("none")} />;

  return (
    <div className="rec:flex rec:flex-col rec:gap-3 rec:p-3">
      {[...active, ...finished].map((j) => (
        <div key={j.id} className="rec:rounded rec:border rec:border-border-default rec:bg-bg-secondary rec:p-2">
          <div className="rec:mb-1 rec:font-mono rec:text-[10px] rec:text-text-tertiary">{j.id}</div>
          <JobPipelineView job={j} />
        </div>
      ))}
    </div>
  );
}
