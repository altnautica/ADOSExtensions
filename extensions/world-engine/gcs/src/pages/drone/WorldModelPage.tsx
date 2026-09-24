/**
 * The drone's World Model page. With a reconstruction available it is the
 * viewer: a session selector and a viewer switcher over the world a captured
 * session reconstructed, sourced local-first from the paired compute node
 * (correlated by session id) with the cloud job records as the fallback.
 * Without one it is the capture setup view. Mounting `useAtlasControl` keeps
 * the drone's capture readiness polled while the page is open.
 */

import { useState } from "react";
import { Boxes } from "lucide-react";

import { ViewerSwitcher } from "../../components/atlas/ViewerSwitcher";
import { WorldGenerationCard } from "../../components/atlas/WorldGenerationCard";
import { WorldModelViewport } from "../../components/atlas/WorldModelViewport";
import {
  DEFAULT_ATLAS_VIEWER,
  backendOf,
  viewerHintOf,
  type AtlasViewer,
} from "../../components/atlas/viewer-types";
import { Select, type SelectOption } from "../../components/ui/Select";
import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { useAtlasControl } from "../../hooks/use-atlas-control";
import { useCloudJobs } from "../../hooks/use-cloud-jobs";
import { useDroneWorldModel } from "../../hooks/use-drone-world-model";
import type { ArtifactSource } from "../../lib/net/artifact-source";
import { useCloudArtifact } from "./use-cloud-artifact";
import { WorldModelSetupView } from "./WorldModelSetupView";

const t = translator("atlas");

export function WorldModelPage() {
  const deviceId = useHost().node.deviceId;
  const control = useAtlasControl(deviceId);

  // Selection is tracked per source (a local session id vs a cloud job id);
  // only one source renders at a time.
  const [localSession, setLocalSession] = useState("");
  const [cloudJobId, setCloudJobId] = useState("");
  const [override, setOverride] = useState<{ key: string; viewer: AtlasViewer } | null>(null);

  const local = useDroneWorldModel({ sessionId: localSession || null, computeNodeId: null });
  const cloudJobs = useCloudJobs(deviceId);
  const useLocal = local.status === "ready" || local.status === "building";

  const latestDone = cloudJobs.find((j) => j.status === "done" && j.outputUrl !== null);
  const selectedJob = useLocal ? null : (cloudJobs.find((j) => j.id === cloudJobId) ?? latestDone ?? null);
  const cloudArtifact = useCloudArtifact(selectedJob);

  let artifact: ArtifactSource | null;
  let hint: AtlasViewer | null;
  // The reconstruction backend for the honesty badge; the cloud path reads it
  // off the job's opaque metadata (null when the producer did not record it).
  let backend: string | null;
  let sessionOptions: SelectOption[];
  let selectedValue: string;
  let onSelect: (v: string) => void;

  if (useLocal) {
    artifact = local.artifact;
    hint = local.viewerHint;
    backend = local.backend;
    sessionOptions = local.sessions.map((s) => ({ value: s.sessionId, label: s.sessionId.slice(0, 12) }));
    selectedValue =
      local.sessions.find((s) => s.sessionId === localSession)?.sessionId ?? local.sessions[0]?.sessionId ?? "";
    onSelect = setLocalSession;
  } else {
    artifact = cloudArtifact;
    hint = selectedJob ? viewerHintOf(selectedJob.metadata) : null;
    backend = selectedJob ? backendOf(selectedJob.metadata) : null;
    sessionOptions = cloudJobs.map((j) => ({
      value: j.id,
      label: `${j.kind} · ${(j.sessionId ?? j.id).slice(0, 8)} · ${j.status}`,
    }));
    selectedValue = selectedJob?.id ?? "";
    onSelect = setCloudJobId;
  }

  const viewer = override && override.key === selectedValue ? override.viewer : (hint ?? DEFAULT_ATLAS_VIEWER);
  // A resolved local output with nothing to read (a placeholder) still opens
  // the viewer so its honesty badge shows.
  const hasReconstruction = artifact !== null || (useLocal && local.status === "ready" && backend !== null);

  if (!hasReconstruction) {
    return (
      <WorldModelSetupView
        control={control}
        local={local}
        noReconstructionPath={!local.hasComputeNode && cloudJobs.length === 0}
      />
    );
  }

  return (
    <div className="we:flex we:h-full we:flex-col">
      <div className="we:flex we:items-center we:gap-2 we:border-b we:border-border-default we:p-3">
        <Boxes className="we:h-4 we:w-4 we:text-text-tertiary" />
        <span className="we:mr-2 we:text-sm we:font-medium we:text-text-primary">{t("worldModelHeading")}</span>
        {sessionOptions.length > 0 && (
          <Select
            options={sessionOptions}
            value={selectedValue}
            onChange={onSelect}
            placeholder={t("worldModelSelectSession")}
            className="we:w-64"
          />
        )}
        <ViewerSwitcher
          viewer={viewer}
          onSelect={(v) => setOverride({ key: selectedValue, viewer: v })}
          ariaLabel={t("viewerGroupLabel")}
        />
      </div>
      <div className="we:flex we:min-h-[320px] we:flex-1">
        <div className="we:relative we:flex-1">
          <WorldModelViewport viewer={viewer} artifact={artifact} backend={backend} />
        </div>
        {/* Which generation is on screen and what it actually contains: the
            viewer shows an artifact, only the descriptors say whether a
            planning input exists for it. */}
        <aside className="we:w-72 we:shrink-0 we:overflow-auto we:border-l we:border-border-default we:p-2">
          <WorldGenerationCard droneDeviceId={control.deviceId} computeNodeDeviceId={local.computeNodeDeviceId} />
        </aside>
      </div>
    </div>
  );
}
