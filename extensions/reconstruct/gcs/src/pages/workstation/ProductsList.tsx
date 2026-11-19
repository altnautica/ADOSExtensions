/**
 * @module ProductsList
 * @description Pick a finished job and preview its reconstructed artifact in
 * the model viewer (Recording / Splat / Cloud / LOD). Outputs are fetched on
 * demand from the compute node and read back through its extension server;
 * calm states cover a job with no artifact yet and a failed read.
 */

import { useEffect, useMemo, useRef, useState } from "react";
import { Boxes } from "lucide-react";

import { ViewerSwitcher } from "@altnautica/mapping-ui/components/model/ViewerSwitcher";
import {
  MODEL_VIEWERS,
  pickArtifactForViewer,
  viewerForKind,
  type ModelViewer,
} from "@altnautica/mapping-ui/components/model/viewer-types";
import { ModelViewport } from "@altnautica/mapping-ui/components/model/ModelViewport";
import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { Select } from "@altnautica/mapping-ui/components/ui/Select";
import { translator } from "../../host/i18n";
import { useOutputFrame } from "@altnautica/mapping-ui/hooks/use-output-frame";
import type { ComputeAgentClient, ComputeJob, ComputeOutput } from "@altnautica/mapping-ui/lib/agent/compute-client";

const t = translator("node");

export function ProductsList({ jobs, client }: { jobs: readonly ComputeJob[]; client: ComputeAgentClient }) {
  // Only finished jobs have artifacts, newest first (the engine lists them
  // oldest first, so an unsorted default would show a stale job).
  const finished = useMemo(
    () => jobs.filter((j) => j.state === "completed").sort((a, b) => b.createdMs - a.createdMs),
    [jobs],
  );
  const [selectedJobId, setSelectedJobId] = useState("");
  // Outputs keyed to their job, so a switch never shows the previous job's
  // artifact while the new read is in flight. `outputs: null` is a failed read,
  // distinct from a reachable job with no artifacts yet (`[]`).
  const [outputState, setOutputState] = useState<{ jobId: string; outputs: ComputeOutput[] | null }>({
    jobId: "",
    outputs: [],
  });
  const [retryNonce, setRetryNonce] = useState(0);
  // The client + job whose non-empty outputs are loaded; once set, the job
  // poll stops re-reading them.
  const loadedRef = useRef<{ client: ComputeAgentClient; jobId: string } | null>(null);
  // A manual viewer choice, keyed to its job: a job switch drops it and the
  // viewer follows the artifact's kind again.
  const [override, setOverride] = useState<{ jobId: string; viewer: ModelViewer } | null>(null);

  // The selection, defaulting to the latest finished job.
  const effectiveJobId = finished.find((j) => j.id === selectedJobId)?.id ?? finished[0]?.id ?? "";

  // Read on a job switch, a manual retry, and every job-poll tick while the
  // outputs are empty or failed (a transient failure or an artifact that lands
  // after the job completes both recover without a switch).
  useEffect(() => {
    if (!effectiveJobId) return;
    const loaded = loadedRef.current;
    if (loaded && loaded.client === client && loaded.jobId === effectiveJobId) return;
    let cancelled = false;
    void client.getOutputs(effectiveJobId).then((res) => {
      if (cancelled) return;
      if (res && res.length > 0) loadedRef.current = { client, jobId: effectiveJobId };
      setOutputState({ jobId: effectiveJobId, outputs: res });
    });
    return () => {
      cancelled = true;
    };
  }, [client, effectiveJobId, jobs, retryNonce]);

  const fetched = outputState.jobId === effectiveJobId;
  const failed = fetched && outputState.outputs === null;
  const outputs = fetched ? (outputState.outputs ?? []) : [];
  // Each viewer reads only the artifact of its own kind (World → `.rrd`,
  // Splat → splat `.ply`, Cloud/LOD → point-cloud `.ply`), so only viewers
  // with a matching output are offered.
  const viewable = MODEL_VIEWERS.filter((v) => pickArtifactForViewer(outputs, v.id) !== undefined);
  const kindViewer = viewerForKind(outputs[0]?.kind ?? "");
  const defaultViewer: ModelViewer = viewable.some((v) => v.id === kindViewer)
    ? kindViewer
    : (viewable[0]?.id ?? kindViewer);
  const viewer =
    override && override.jobId === effectiveJobId && viewable.some((v) => v.id === override.viewer)
      ? override.viewer
      : defaultViewer;
  const picked = pickArtifactForViewer(outputs, viewer);
  const frame = useOutputFrame(client, picked, outputs);
  const showable = picked !== undefined && picked.source !== null && frame !== undefined;

  if (finished.length === 0) {
    return <div className="rec:text-[11px] rec:text-text-tertiary rec:text-center rec:py-8">{t("forgeNoOutputs")}</div>;
  }

  let emptyMessage: string;
  if (!fetched) emptyMessage = t("forgeOutputsLoading");
  else if (failed) emptyMessage = t("forgeOutputsFailed");
  else if (outputs.length > 0) emptyMessage = t("forgeNoViewableArtifact");
  else emptyMessage = t("forgeNoOutputs");

  return (
    <div className="rec:flex rec:flex-col rec:h-full">
      <div className="rec:flex rec:items-center rec:gap-2 rec:p-2 rec:border-b rec:border-border-default">
        <Select
          options={finished.map((j) => ({ value: j.id, label: `${j.kind} · ${j.id}` }))}
          value={effectiveJobId}
          onChange={setSelectedJobId}
          placeholder={t("forgeSelectJob")}
          className="rec:w-56"
        />
        {viewable.length > 0 && (
          <ViewerSwitcher
            viewer={viewer}
            viewers={viewable}
            onSelect={(v) => setOverride({ jobId: effectiveJobId, viewer: v })}
            ariaLabel={t("forgeOutputs")}
          />
        )}
      </div>

      <div className="rec:flex-1 rec:relative rec:min-h-[320px]">
        {showable ? (
          <ModelViewport viewer={viewer} artifact={picked.source} backend={picked.backend} frame={frame} />
        ) : (
          <div className="rec:absolute rec:inset-0 rec:flex rec:items-center rec:justify-center rec:p-6">
            <div className="rec:text-center">
              <Boxes className="rec:w-5 rec:h-5 rec:text-text-tertiary rec:mx-auto rec:mb-2" />
              <p className="rec:text-[11px] rec:text-text-tertiary rec:max-w-xs">{emptyMessage}</p>
              {failed && (
                <Button
                  size="sm"
                  variant="secondary"
                  className="rec:mt-2"
                  onClick={() => setRetryNonce((n) => n + 1)}
                >
                  {t("forgeOutputsRetry")}
                </Button>
              )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
