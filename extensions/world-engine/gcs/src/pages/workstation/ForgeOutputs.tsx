/**
 * @module ForgeOutputs
 * @description Pick a finished job and preview its reconstructed artifact in
 * the World Model viewer (World / Splat / Cloud / LOD). Outputs are fetched on
 * demand from the compute node and read back through its extension server;
 * calm states cover a job with no artifact yet and a failed read.
 */

import { useEffect, useMemo, useRef, useState } from "react";
import { Boxes } from "lucide-react";

import { ViewerSwitcher } from "../../components/atlas/ViewerSwitcher";
import {
  ATLAS_VIEWERS,
  pickArtifactForViewer,
  viewerForKind,
  type AtlasViewer,
} from "../../components/atlas/viewer-types";
import { WorldModelViewport } from "../../components/atlas/WorldModelViewport";
import { Button } from "../../components/ui/Button";
import { Select } from "../../components/ui/Select";
import { translator } from "../../host/i18n";
import type { ComputeAgentClient, ComputeJob, ComputeOutput } from "../../lib/agent/compute-client";

const t = translator("atlas");

export function ForgeOutputs({ jobs, client }: { jobs: readonly ComputeJob[]; client: ComputeAgentClient }) {
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
  const [override, setOverride] = useState<{ jobId: string; viewer: AtlasViewer } | null>(null);

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
  const viewable = ATLAS_VIEWERS.filter((v) => pickArtifactForViewer(outputs, v.id) !== undefined);
  const kindViewer = viewerForKind(outputs[0]?.kind ?? "");
  const defaultViewer: AtlasViewer = viewable.some((v) => v.id === kindViewer)
    ? kindViewer
    : (viewable[0]?.id ?? kindViewer);
  const viewer =
    override && override.jobId === effectiveJobId && viewable.some((v) => v.id === override.viewer)
      ? override.viewer
      : defaultViewer;
  const picked = pickArtifactForViewer(outputs, viewer);
  // A placeholder artifact has no readable source but still badges its
  // backend; an output with neither has nothing to show.
  const showable = picked !== undefined && (picked.source !== null || picked.backend !== null);

  if (finished.length === 0) {
    return <div className="we:text-[11px] we:text-text-tertiary we:text-center we:py-8">{t("forgeNoOutputs")}</div>;
  }

  let emptyMessage: string;
  if (!fetched) emptyMessage = t("forgeOutputsLoading");
  else if (failed) emptyMessage = t("forgeOutputsFailed");
  else if (outputs.length > 0) emptyMessage = t("forgeNoViewableArtifact");
  else emptyMessage = t("forgeNoOutputs");

  return (
    <div className="we:flex we:flex-col we:h-full">
      <div className="we:flex we:items-center we:gap-2 we:p-2 we:border-b we:border-border-default">
        <Select
          options={finished.map((j) => ({ value: j.id, label: `${j.kind} · ${j.id}` }))}
          value={effectiveJobId}
          onChange={setSelectedJobId}
          placeholder={t("forgeSelectJob")}
          className="we:w-56"
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

      <div className="we:flex-1 we:relative we:min-h-[320px]">
        {showable ? (
          <WorldModelViewport viewer={viewer} artifact={picked.source} backend={picked.backend} />
        ) : (
          <div className="we:absolute we:inset-0 we:flex we:items-center we:justify-center we:p-6">
            <div className="we:text-center">
              <Boxes className="we:w-5 we:h-5 we:text-text-tertiary we:mx-auto we:mb-2" />
              <p className="we:text-[11px] we:text-text-tertiary we:max-w-xs">{emptyMessage}</p>
              {failed && (
                <Button
                  size="sm"
                  variant="secondary"
                  className="we:mt-2"
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
