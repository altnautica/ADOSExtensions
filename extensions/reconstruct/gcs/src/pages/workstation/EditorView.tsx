/**
 * @module EditorView
 * @description The Editor segment: a finished job's point cloud, opened
 * read-only in the level-of-detail viewer. Picking a cloud is the whole
 * surface for now; editing tools and the provenance column come with the
 * editor.
 */

import { useEffect, useMemo, useState } from "react";
import { Boxes } from "lucide-react";

import { PointCloudLodViewer } from "@altnautica/mapping-ui/components/model/viewers/PointCloudLodViewer";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";
import { Select } from "@altnautica/mapping-ui/components/ui/Select";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";
import type { ComputeOutput } from "@altnautica/mapping-ui/lib/agent/compute-client";

import { translator } from "../../host/i18n";

const t = translator("editor");

/** A cloud a viewer can read. */
function isCloud(o: ComputeOutput): boolean {
  return o.source !== null && (o.kind === "cloud" || o.kind === "pointcloud" || o.format === "laz");
}

export function EditorView({ deviceId }: { deviceId: string | null }) {
  const { jobs, client } = useComputeJobs(deviceId);
  const finished = useMemo(
    () => jobs.filter((j) => j.state === "completed").sort((a, b) => b.createdMs - a.createdMs),
    [jobs],
  );
  const [jobId, setJobId] = useState("");
  const effective = finished.find((j) => j.id === jobId)?.id ?? finished[0]?.id ?? "";
  const [clouds, setClouds] = useState<{ jobId: string; outputs: ComputeOutput[] }>({ jobId: "", outputs: [] });

  useEffect(() => {
    if (!client || !effective) return;
    let cancelled = false;
    void client.getOutputs(effective).then((outs) => {
      if (!cancelled) setClouds({ jobId: effective, outputs: (outs ?? []).filter(isCloud) });
    });
    return () => {
      cancelled = true;
    };
  }, [client, effective]);

  if (!client) return <CalmState icon={Boxes} message={t("localOnly")} />;
  if (finished.length === 0) return <CalmState icon={Boxes} message={t("noJobs")} />;
  const cloud = clouds.jobId === effective ? clouds.outputs[0] : undefined;

  return (
    <div className="rec:flex rec:h-full rec:flex-col">
      <div className="rec:flex rec:items-center rec:gap-2 rec:border-b rec:border-border-default rec:p-3">
        <Select
          options={finished.map((j) => ({ value: j.id, label: `${j.kind} · ${(j.sessionId ?? j.id).slice(0, 12)}` }))}
          value={effective}
          onChange={setJobId}
          placeholder={t("pickJob")}
          className="rec:w-72"
        />
      </div>
      <div className="rec:relative rec:min-h-[320px] rec:flex-1">
        {cloud?.source ? (
          <PointCloudLodViewer source={cloud.source} />
        ) : (
          <CalmState icon={Boxes} message={t("noCloud")} />
        )}
      </div>
    </div>
  );
}
