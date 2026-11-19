/**
 * A drone's Live view: a how-it-works explainer, the requirements checklist
 * (cameras / compute node reachable / capture service), and the enable,
 * capture-profile, detail-level and capture lifecycle controls, each disabled
 * with its reason until it can apply.
 */

import { Boxes, Camera, Cpu, Eye, Video } from "lucide-react";

import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { Select, type SelectOption } from "@altnautica/mapping-ui/components/ui/Select";
import { translator } from "../../host/i18n";
import type { CaptureControl } from "../../hooks/use-capture-control";
import type { NodeModel } from "../../hooks/use-node-model";
import { computeCaptureGate } from "../../lib/capture/requirements";
import { DroneCaptureControls } from "../setup/DroneCaptureControls";
import { RequirementsChecklist } from "../setup/RequirementsChecklist";
import { ReconstructQualitySelect } from "../setup/ReconstructQualitySelect";

const t = translator("setup");

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
  auto: "capture.poseAuto",
  fc_ekf: "capture.poseFcEkf",
  navigation: "capture.poseNavigation",
};

function InfoRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="cap:flex cap:items-center cap:justify-between cap:text-[11px]">
      <span className="cap:text-text-tertiary">{label}</span>
      <span className="cap:font-mono cap:tabular-nums cap:text-text-secondary">{value}</span>
    </div>
  );
}

function HowStep({ icon: Icon, title, body }: { icon: typeof Camera; title: string; body: string }) {
  return (
    <div className="cap:flex cap:items-start cap:gap-2">
      <Icon className="cap:mt-0.5 cap:h-4 cap:w-4 cap:shrink-0 cap:text-accent-primary" />
      <div>
        <div className="cap:text-[11px] cap:font-medium cap:text-text-primary">{title}</div>
        <div className="cap:text-[10px] cap:text-text-tertiary">{body}</div>
      </div>
    </div>
  );
}

export function DroneLiveView({ control, local }: { control: CaptureControl; local: NodeModel }) {
  const noReconstructionPath = !local.hasComputeNode;
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
    <div className="cap:min-h-0 cap:flex-1 cap:overflow-auto">
      <div className="cap:space-y-4 cap:p-4">
        <div>
          <div className="cap:flex cap:items-center cap:gap-2">
            <Boxes className="cap:h-4 cap:w-4 cap:text-accent-primary" />
            <h2 className="cap:text-sm cap:font-semibold cap:text-text-primary">{t("capture.setupTitle")}</h2>
          </div>
          <p className="cap:mt-1 cap:max-w-2xl cap:text-[11px] cap:text-text-tertiary">{t("capture.setupIntro")}</p>
        </div>

        <div className="cap:grid cap:grid-cols-1 cap:gap-4 cap:xl:grid-cols-2">
          <div className="cap:space-y-3">
            <div className="cap:space-y-2 cap:rounded-lg cap:border cap:border-border-default cap:p-3">
              <div className="cap:flex cap:items-center cap:gap-1.5">
                <Video className="cap:h-3.5 cap:w-3.5 cap:text-text-tertiary" />
                <span className="cap:text-xs cap:font-medium cap:text-text-secondary">
                  {t("capture.liveStreamTitle")}
                </span>
              </div>
              <p className="cap:text-[11px] cap:text-text-tertiary">{t("capture.liveStreamNote")}</p>
            </div>

            <div className="cap:space-y-2.5 cap:rounded-lg cap:border cap:border-border-default cap:p-3">
              <span className="cap:text-xs cap:font-medium cap:text-text-secondary">{t("capture.howItWorksTitle")}</span>
              <HowStep icon={Camera} title={t("capture.howStep1Title")} body={t("capture.howStep1Body")} />
              <HowStep icon={Cpu} title={t("capture.howStep2Title")} body={t("capture.howStep2Body")} />
              <HowStep icon={Eye} title={t("capture.howStep3Title")} body={t("capture.howStep3Body")} />
            </div>
          </div>

          <div className="cap:space-y-3">
            <div className="cap:rounded-lg cap:border cap:border-border-default cap:p-3">
              <RequirementsChecklist requirements={gate.requirements} />
            </div>

            <div className="cap:space-y-3 cap:rounded-lg cap:border cap:border-border-default cap:p-3">
              <span className="cap:text-xs cap:font-medium cap:text-text-secondary">
                {t("capture.captureControlsTitle")}
              </span>

              {readiness && (
                <div className="cap:space-y-1.5">
                  <InfoRow label={t("capture.camerasLabel")} value={String(readiness.camerasConfigured)} />
                  <InfoRow
                    label={t("capture.poseSourceLabel")}
                    value={t(POSE_SOURCE_KEY[readiness.poseSource] ?? "capture.poseAuto")}
                  />
                  <div className="cap:flex cap:items-center cap:justify-between cap:gap-2 cap:text-[11px]">
                    <span className="cap:text-text-tertiary">{t("capture.profileLabel")}</span>
                    <div className="cap:w-40">
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

              <div className="cap:flex cap:flex-wrap cap:items-center cap:gap-2">
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
                <p role="alert" className="cap:text-[10px] cap:text-status-error">
                  {control.configError}
                </p>
              )}

              <DroneCaptureControls
                control={control}
                canStart={gate.canStart}
                startBlockedKey={gate.startBlockedKey}
                blockedKey={commandable ? null : "capture.notLocalReason"}
              />

              {readiness?.capturing === true && (
                <p className="cap:text-[10px] cap:text-accent-primary">{t("capture.capturingHint")}</p>
              )}
              {noReconstructionPath && (
                <p className="cap:text-[10px] cap:text-text-tertiary">{t("worldModelNoNode")}</p>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
