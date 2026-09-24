/**
 * The World Model page before any reconstruction exists: a how-it-works
 * explainer, the requirements checklist (cameras / compute node reachable /
 * capture service), and the enable, capture-profile, detail-level and capture
 * lifecycle controls, each disabled with its reason until it can apply.
 */

import { Boxes, Camera, Cpu, Eye, Video } from "lucide-react";

import { WorldGenerationCard } from "../../components/atlas/WorldGenerationCard";
import { Button } from "../../components/ui/Button";
import { Select, type SelectOption } from "../../components/ui/Select";
import { translator } from "../../host/i18n";
import type { AtlasControl } from "../../hooks/use-atlas-control";
import type { DroneWorldModel } from "../../hooks/use-drone-world-model";
import { computeCaptureGate } from "../../lib/atlas/capture-requirements";
import { AtlasCaptureControls } from "./atlas/AtlasCaptureControls";
import { AtlasRequirementsChecklist } from "./atlas/AtlasRequirementsChecklist";
import { ReconstructQualitySelect } from "./atlas/ReconstructQualitySelect";

const t = translator("atlas");

/** The capture profiles the agent's strict capture-profile enum accepts. An
 * unknown value the agent reports is kept as its own option so the picker
 * never drops it. */
const CAPTURE_PROFILES: readonly SelectOption[] = [
  { value: "orbit", label: t("capture.profileOrbit") },
  { value: "lawnmower", label: t("capture.profileLawnmower") },
  { value: "freeform", label: t("capture.profileFreeform") },
  { value: "inspection", label: t("capture.profileInspection") },
];

const POSE_SOURCE_KEY: Record<string, string> = {
  offloaded_slam: "capture.poseOffloadedSlam",
  hybrid: "capture.poseHybrid",
};

function InfoRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="we:flex we:items-center we:justify-between we:text-[11px]">
      <span className="we:text-text-tertiary">{label}</span>
      <span className="we:font-mono we:tabular-nums we:text-text-secondary">{value}</span>
    </div>
  );
}

function HowStep({ icon: Icon, title, body }: { icon: typeof Camera; title: string; body: string }) {
  return (
    <div className="we:flex we:items-start we:gap-2">
      <Icon className="we:mt-0.5 we:h-4 we:w-4 we:shrink-0 we:text-accent-primary" />
      <div>
        <div className="we:text-[11px] we:font-medium we:text-text-primary">{title}</div>
        <div className="we:text-[10px] we:text-text-tertiary">{body}</div>
      </div>
    </div>
  );
}

export function WorldModelSetupView({
  control,
  local,
  noReconstructionPath,
}: {
  control: AtlasControl;
  local: DroneWorldModel;
  /** No compute node is paired and no cloud job record exists: nothing can
   * reconstruct this drone's capture yet. */
  noReconstructionPath: boolean;
}) {
  const readiness = control.readiness;
  const commandable = control.live;
  const gate = computeCaptureGate({
    readiness,
    computePaired: local.hasComputeNode,
    computeReachable: local.status === "ready" || local.status === "building",
  });
  const enabled = readiness?.enabled === true;
  const captureProfile = readiness?.captureProfile || "freeform";
  const profileOptions = CAPTURE_PROFILES.some((p) => p.value === captureProfile)
    ? CAPTURE_PROFILES
    : [{ value: captureProfile, label: captureProfile }, ...CAPTURE_PROFILES];

  return (
    <div className="we:min-h-0 we:flex-1 we:overflow-auto">
      <div className="we:space-y-4 we:p-4">
        <div>
          <div className="we:flex we:items-center we:gap-2">
            <Boxes className="we:h-4 we:w-4 we:text-accent-primary" />
            <h2 className="we:text-sm we:font-semibold we:text-text-primary">{t("capture.setupTitle")}</h2>
          </div>
          <p className="we:mt-1 we:max-w-2xl we:text-[11px] we:text-text-tertiary">{t("capture.setupIntro")}</p>
        </div>

        <div className="we:grid we:grid-cols-1 we:gap-4 we:xl:grid-cols-2">
          <div className="we:space-y-3">
            <div className="we:space-y-2 we:rounded-lg we:border we:border-border-default we:p-3">
              <div className="we:flex we:items-center we:gap-1.5">
                <Video className="we:h-3.5 we:w-3.5 we:text-text-tertiary" />
                <span className="we:text-xs we:font-medium we:text-text-secondary">
                  {t("capture.liveStreamTitle")}
                </span>
              </div>
              <p className="we:text-[11px] we:text-text-tertiary">{t("capture.liveStreamNote")}</p>
            </div>

            <div className="we:space-y-2.5 we:rounded-lg we:border we:border-border-default we:p-3">
              <span className="we:text-xs we:font-medium we:text-text-secondary">{t("capture.howItWorksTitle")}</span>
              <HowStep icon={Camera} title={t("capture.howStep1Title")} body={t("capture.howStep1Body")} />
              <HowStep icon={Cpu} title={t("capture.howStep2Title")} body={t("capture.howStep2Body")} />
              <HowStep icon={Eye} title={t("capture.howStep3Title")} body={t("capture.howStep3Body")} />
            </div>
          </div>

          <div className="we:space-y-3">
            <WorldGenerationCard droneDeviceId={control.deviceId} computeNodeDeviceId={local.computeNodeDeviceId} />
            <div className="we:rounded-lg we:border we:border-border-default we:p-3">
              <AtlasRequirementsChecklist requirements={gate.requirements} />
            </div>

            <div className="we:space-y-3 we:rounded-lg we:border we:border-border-default we:p-3">
              <span className="we:text-xs we:font-medium we:text-text-secondary">
                {t("capture.captureControlsTitle")}
              </span>

              {readiness && (
                <div className="we:space-y-1.5">
                  <InfoRow label={t("capture.camerasLabel")} value={String(readiness.camerasConfigured)} />
                  <InfoRow
                    label={t("capture.poseSourceLabel")}
                    value={t(POSE_SOURCE_KEY[readiness.poseSource] ?? "capture.poseLocalVio")}
                  />
                  <div className="we:flex we:items-center we:justify-between we:gap-2 we:text-[11px]">
                    <span className="we:text-text-tertiary">{t("capture.profileLabel")}</span>
                    <div className="we:w-40">
                      <Select
                        options={profileOptions}
                        value={captureProfile}
                        onChange={(v) => void control.setCaptureProfile(v)}
                        disabled={!commandable || control.busy}
                      />
                    </div>
                  </div>
                  <ReconstructQualitySelect control={control} />
                </div>
              )}

              <div className="we:flex we:flex-wrap we:items-center we:gap-2">
                <Button
                  size="sm"
                  variant={enabled ? "secondary" : "primary"}
                  loading={control.busy}
                  disabled={!commandable || control.busy}
                  title={!commandable ? t("capture.notLocalReason") : undefined}
                  onClick={() => void (enabled ? control.disable() : control.enable())}
                >
                  {enabled ? t("capture.disableCapture") : t("capture.enableCapture")}
                </Button>
              </div>
              {control.configError && (
                <p role="alert" className="we:text-[10px] we:text-status-error">
                  {control.configError}
                </p>
              )}

              <AtlasCaptureControls
                control={control}
                canStart={gate.canStart}
                startBlockedKey={gate.startBlockedKey}
                blockedKey={commandable ? null : "capture.notLocalReason"}
              />

              {readiness?.capturing === true && (
                <p className="we:text-[10px] we:text-accent-primary">{t("capture.capturingHint")}</p>
              )}
              {noReconstructionPath && (
                <p className="we:text-[10px] we:text-text-tertiary">{t("worldModelNoNode")}</p>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
