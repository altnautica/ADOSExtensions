/**
 * The drone's world-model setup page: the capture-service master switch, the
 * pose-source preference (plugin config `atlas.pose_tier`), and the capture
 * profile and reconstruction detail shown read-only. The World Model page owns
 * those two writes, next to the capture controls, so this page never runs a
 * second writer for the same keys.
 */

import { Boxes } from "lucide-react";

import {
  ConfigReadonlyRow,
  ConfigSelectField,
  InfoNote,
  Section,
  usePluginConfig,
} from "../../components/config/ConfigFields";
import { useToast } from "../../components/ui/Toast";
import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { useAtlasControl } from "../../hooks/use-atlas-control";
import { cn } from "../../lib/utils";

const t = translator("nodeSettings.atlas");
const tSettings = translator("nodeSettings");

const CAPTURE_PROFILE_LABELS: Record<string, string> = {
  orbit: t("profileOrbit"),
  lawnmower: t("profileLawnmower"),
  freeform: t("profileFreeform"),
  inspection: t("profileInspection"),
};

const POSE_TIER_OPTIONS = [
  { value: "auto", label: t("poseTierAuto") },
  { value: "local", label: t("poseTierLocal") },
  { value: "offload", label: t("poseTierOffload") },
  { value: "hybrid", label: t("poseTierHybrid") },
];

/** The capture-service switch: on/off follows the drone's reported readiness;
 * unknown (no readiness yet, or the node stopped answering) is disabled rather
 * than drawn as off. */
function CaptureServiceSwitch({ deviceId }: { deviceId: string | null }) {
  const control = useAtlasControl(deviceId);
  const { toast } = useToast();
  const known = control.readiness !== null;
  const checked = control.readiness?.enabled === true;
  const disabled = !control.live || !known || control.busy;

  const onToggle = async () => {
    if (disabled) return;
    const ok = checked ? await control.disable() : await control.enable();
    toast(ok ? tSettings("applied") : tSettings("applyFailed"), ok ? "success" : "error");
  };

  return (
    <div className="we:flex we:flex-col we:items-end we:gap-1">
      <button
        type="button"
        role="switch"
        aria-checked={checked}
        aria-label={t("featureLabel")}
        disabled={disabled}
        aria-busy={control.busy || undefined}
        onClick={() => void onToggle()}
        className={cn(
          "we:relative we:inline-flex we:h-5 we:w-9 we:shrink-0 we:items-center we:rounded-full we:transition-colors",
          "we:focus-visible:outline-2 we:focus-visible:outline-accent-primary",
          "we:disabled:cursor-not-allowed we:disabled:opacity-50",
          checked ? "we:bg-accent-primary" : "we:bg-bg-tertiary we:border we:border-border-default",
        )}
      >
        <span
          className={cn(
            "we:inline-block we:h-3.5 we:w-3.5 we:rounded-full we:bg-text-primary we:transition-transform",
            checked ? "we:translate-x-[18px]" : "we:translate-x-[3px]",
          )}
        />
      </button>
      {!known && <span className="we:text-[10px] we:text-text-tertiary">{tSettings("notReported")}</span>}
      {control.configError && (
        <p role="alert" className="we:max-w-56 we:text-right we:text-[10px] we:text-status-error">
          {control.configError}
        </p>
      )}
    </div>
  );
}

export function WorldModelSetupPage() {
  const deviceId = useHost().node.deviceId;
  const { config, setValue, error } = usePluginConfig(deviceId);
  const readOnly = config === null;

  return (
    <div className="we:min-h-0 we:flex-1 we:overflow-auto we:p-4">
      <Section title={t("title")} icon={Boxes} blurb={t("blurb")}>
        <div className="we:flex we:items-start we:justify-between we:gap-3">
          <div className="we:min-w-0">
            <div className="we:text-xs we:font-medium we:text-text-primary">{t("featureLabel")}</div>
            <p className="we:mt-0.5 we:text-[11px] we:text-text-tertiary">{t("featureHint")}</p>
          </div>
          <div className="we:shrink-0 we:pt-0.5">
            <CaptureServiceSwitch deviceId={deviceId} />
          </div>
        </div>

        {error && <InfoNote>{error}</InfoNote>}

        <div className="we:border-t we:border-border-default we:pt-3">
          <ConfigSelectField
            configKey="atlas.pose_tier"
            label={t("poseTierLabel")}
            hint={t("poseTierHint")}
            options={POSE_TIER_OPTIONS}
            config={config}
            readOnly={readOnly}
            setValue={setValue}
          />
        </div>

        <div className="we:space-y-2 we:border-t we:border-border-default we:pt-3">
          <ConfigReadonlyRow
            configKey="atlas.capture_profile"
            label={t("captureProfileLabel")}
            config={config}
            format={(raw) =>
              typeof raw === "string" && raw.length > 0 ? (CAPTURE_PROFILE_LABELS[raw] ?? raw) : null
            }
          />
          <ConfigReadonlyRow configKey="atlas.reconstruct_steps" label={t("reconstructStepsLabel")} config={config} />
          <p className="we:text-[11px] we:text-text-tertiary">{t("captureManagedHint")}</p>
        </div>
      </Section>
    </div>
  );
}
