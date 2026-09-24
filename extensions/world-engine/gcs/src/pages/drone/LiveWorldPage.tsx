/**
 * The drone's in-flight Live World page: the monitor of a world-model capture
 * session (capture stats, the building reconstruction, the paired
 * reconstructor, the active transport bearer) plus the operator capture
 * controls (Start / Pause / Resume / Stop & Reconstruct and a manual
 * "Reconstruct now"). Reads the drone's live capture slice and drives capture
 * through `useAtlasControl`.
 */

import { useState } from "react";
import { Boxes, Clock } from "lucide-react";

import { ViewerSwitcher } from "../../components/atlas/ViewerSwitcher";
import { WorldModelViewport } from "../../components/atlas/WorldModelViewport";
import {
  DEFAULT_ATLAS_VIEWER,
  backendOf,
  viewerHintOf,
  type AtlasViewer,
} from "../../components/atlas/viewer-types";
import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { useAtlasControl } from "../../hooks/use-atlas-control";
import { useCloudJobs } from "../../hooks/use-cloud-jobs";
import { useDroneWorldModel } from "../../hooks/use-drone-world-model";
import { DEFAULT_RECONSTRUCTION_STEPS, qualityForSteps } from "../../lib/atlas/reconstruction-quality";
import { useNow } from "../../lib/clock";
import { cn } from "../../lib/utils";
import { EMPTY_ATLAS_LIVE, useAtlasStore } from "../../stores/atlas-store";
import { AtlasCaptureControls } from "./atlas/AtlasCaptureControls";
import { ReconstructQualitySelect } from "./atlas/ReconstructQualitySelect";
import { LiveWorldCards } from "./LiveWorldCards";
import { useCloudArtifact } from "./use-cloud-artifact";

const t = translator("atlas");

/** The capture fields ride the agent's state feed (seconds cadence), so the
 * staleness budget is heartbeat-scaled. */
const STALE_MS = 15_000;

/** The agent's capture-state vocabulary, plus forward-looking states. */
const STATE_TONE: Record<string, string> = {
  capturing: "we:text-status-success",
  active: "we:text-status-success",
  finalizing: "we:text-accent-primary",
  ready: "we:text-accent-primary",
  paused: "we:text-status-warning",
  bagged: "we:text-text-tertiary",
  ended: "we:text-text-tertiary",
  idle: "we:text-text-tertiary",
  error: "we:text-status-error",
};

export function LiveWorldPage() {
  const deviceId = useHost().node.deviceId;
  const live = useAtlasStore((s) => (deviceId ? s.live[deviceId] : undefined) ?? EMPTY_ATLAS_LIVE);
  const now = useNow();
  const control = useAtlasControl(deviceId);
  const [override, setOverride] = useState<{ sessionKey: string; viewer: AtlasViewer } | null>(null);

  // "{n}s ago" / "{n}m ago" from an epoch-ms timestamp, or null when absent.
  const ago = (tsMs: number | null): string | null => {
    if (tsMs === null) return null;
    const s = Math.max(0, Math.round((now - tsMs) / 1000));
    return s < 60 ? t("capture.agoSeconds", { s }) : t("capture.agoMinutes", { m: Math.round(s / 60) });
  };

  // Local first: the newest completed reconstruction for the active session on
  // the paired compute node, whose client also submits "Reconstruct now".
  const local = useDroneWorldModel({ sessionId: live.sessionId, computeNodeId: live.computeNodeId });

  // Cloud fallback: the newest completed job record for the active session.
  const cloudJobs = useCloudJobs(deviceId);
  const cloudDone =
    (live.sessionId ? cloudJobs.filter((j) => j.sessionId === live.sessionId) : cloudJobs).find(
      (j) => j.status === "done" && j.outputUrl !== null,
    ) ?? null;
  const cloudArtifact = useCloudArtifact(cloudDone);

  const commandable = control.live;
  const reconstructAvailable = local.computeClient !== null && live.sessionId !== null;
  const reconstructSubmit = async (): Promise<boolean> => {
    const client = local.computeClient;
    if (!client || !live.sessionId) return false;
    // The capturing drone's device id lets the compute node attribute the job
    // and record it in the cloud like an automatic reconstruct. The operator's
    // detail level maps to the training step count plus the gaussian cap and SH
    // degree that bound its training time.
    const steps = control.readiness?.reconstructSteps ?? DEFAULT_RECONSTRUCTION_STEPS;
    const quality = qualityForSteps(steps);
    const params: Record<string, unknown> = {
      session_id: live.sessionId,
      steps,
      max_splats: quality.maxSplats,
      sh_degree: quality.shDegree,
    };
    if (control.deviceId) params.device_id = control.deviceId;
    const res = await client.submitJob({ kind: "reconstruct", params });
    return res !== null;
  };

  const sessionKey = live.sessionId ?? "";
  const useLocal = local.status === "ready" || local.status === "building";
  const artifact = useLocal ? local.artifact : cloudArtifact;
  const viewerHint = useLocal ? local.viewerHint : cloudDone ? viewerHintOf(cloudDone.metadata) : null;
  const backend = useLocal ? local.backend : cloudDone ? backendOf(cloudDone.metadata) : null;
  const hasWorld = artifact !== null || backend !== null;
  // A paired but unreachable reconstructor reads distinctly from one that is
  // building; "no node" guidance shows only when nothing can reconstruct.
  const worldEmptyMessage =
    local.hasComputeNode && local.status === "unreachable"
      ? t("capture.reconstructorUnreachable")
      : !local.hasComputeNode && cloudJobs.length === 0
        ? t("worldModelNoNode")
        : t("liveWorldBuilding");

  const viewer =
    override && override.sessionKey === sessionKey ? override.viewer : (viewerHint ?? DEFAULT_ATLAS_VIEWER);

  // A frozen heartbeat never reads as live: past the budget it is badged stale
  // and the rates are dimmed.
  const hasLive = live.state !== null;
  const isStale = live.updatedAt === null || now - live.updatedAt > STALE_MS;
  const stateLabel = live.state ?? control.readiness?.state ?? "idle";
  const stateTone = isStale ? "we:text-status-warning" : (STATE_TONE[stateLabel] ?? "we:text-text-secondary");
  const sessionId = live.sessionId ?? control.readiness?.sessionId ?? null;

  return (
    <div className="we:min-h-0 we:flex-1 we:overflow-auto">
      <div className="we:space-y-4 we:p-4">
        <div className="we:flex we:items-center we:justify-between we:rounded-lg we:border we:border-border-default we:p-3">
          <div className="we:flex we:items-center we:gap-2">
            <Boxes className="we:h-4 we:w-4 we:text-text-tertiary" />
            <span className="we:text-sm we:font-medium we:text-text-primary">{t("capture.liveWorldTitle")}</span>
            <span
              className={cn(
                "we:rounded we:bg-text-primary/[0.04] we:px-1.5 we:py-0.5 we:text-[10px] we:font-medium",
                stateTone,
              )}
            >
              {isStale && hasLive ? t("stale") : stateLabel}
            </span>
            {isStale && hasLive && (
              <span className="we:flex we:items-center we:gap-1 we:text-[10px] we:text-status-warning">
                <Clock className="we:h-3 we:w-3" />
                {ago(live.updatedAt) ?? t("capture.noHeartbeat")}
              </span>
            )}
          </div>
          <span
            className="we:max-w-[40%] we:truncate we:font-mono we:text-[10px] we:text-text-tertiary"
            title={sessionId ?? undefined}
          >
            {sessionId ?? t("capture.noSession")}
          </span>
        </div>

        <div className="we:space-y-2 we:rounded-lg we:border we:border-border-default we:p-3">
          <span className="we:text-xs we:font-medium we:text-text-secondary">{t("capture.captureControlsTitle")}</span>
          <ReconstructQualitySelect control={control} />
          <AtlasCaptureControls
            control={control}
            canStart={commandable}
            startBlockedKey={null}
            blockedKey={commandable ? null : "capture.notLocalReason"}
            reconstruct={{
              available: reconstructAvailable,
              disabledKey: reconstructAvailable
                ? null
                : live.sessionId
                  ? "capture.reconstructDisabledNoNode"
                  : "capture.reconstructDisabledNoSession",
              submit: reconstructSubmit,
            }}
          />
        </div>

        {hasLive ? (
          <LiveWorldCards live={live} stale={isStale} lastKeyframeAgo={ago(live.lastKfAt)} />
        ) : (
          <div className="we:rounded-lg we:border we:border-border-default we:p-6 we:text-center">
            <Boxes className="we:mx-auto we:mb-2 we:h-5 we:w-5 we:animate-pulse we:text-text-tertiary" />
            <p className="we:text-[11px] we:text-text-tertiary">{t("capture.awaitingTelemetry")}</p>
          </div>
        )}

        {/* The newest completed reconstruction for the active session,
            refreshing in place as periodic reconstruct cycles land. */}
        <div className="we:overflow-hidden we:rounded-lg we:border we:border-border-default">
          <div className="we:flex we:items-center we:gap-2 we:border-b we:border-border-default we:px-3 we:py-2">
            <Boxes className="we:h-3.5 we:w-3.5 we:text-text-tertiary" />
            <span className="we:text-xs we:font-medium we:text-text-secondary">{t("capture.worldModelSection")}</span>
            {hasWorld && (
              <ViewerSwitcher
                viewer={viewer}
                onSelect={(v) => setOverride({ sessionKey, viewer: v })}
                ariaLabel={t("viewerGroupLabel")}
              />
            )}
          </div>
          <div className="we:relative we:h-[420px]">
            {hasWorld ? (
              <WorldModelViewport viewer={viewer} artifact={artifact} backend={backend} />
            ) : (
              <div className="we:absolute we:inset-0 we:flex we:items-center we:justify-center we:p-6">
                <div className="we:text-center">
                  <Boxes className="we:mx-auto we:mb-2 we:h-5 we:w-5 we:animate-pulse we:text-text-tertiary" />
                  <p className="we:max-w-xs we:text-[11px] we:text-text-tertiary">{worldEmptyMessage}</p>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
