/**
 * Capture setup (an Agent page on a drone or Visor): the capture-service
 * master switch, the pose source (plugin config `capture.pose_source`), and
 * the capture profile and reconstruction detail shown read-only. The Capture
 * surface owns those two writes, next to the capture controls, so this page
 * never runs a second writer for the same keys.
 */

import { Boxes } from "lucide-react";

import {
  ConfigReadonlyRow,
  ConfigSelectField,
  InfoNote,
  Section,
  usePluginConfig,
} from "@altnautica/mapping-ui/components/config/ConfigFields";
import { useToast } from "@altnautica/mapping-ui/components/ui/Toast";
import { useHost } from "@altnautica/mapping-ui/host/context";
import { translator } from "../host/i18n";
import { useCaptureControl } from "../hooks/use-capture-control";
import { cn } from "@altnautica/mapping-ui/lib/utils";

const t = translator("nodeSettings.capture");
const tSettings = translator("nodeSettings");

const CAPTURE_PROFILE_LABELS: Record<string, string> = {
  orbit: t("profileOrbit"),
  lawnmower: t("profileLawnmower"),
  freeform: t("profileFreeform"),
  inspection: t("profileInspection"),
};

const POSE_SOURCE_OPTIONS = [
  { value: "auto", label: t("poseTierAuto") },
  { value: "fc_ekf", label: t("poseTierFcEkf") },
  { value: "navigation", label: t("poseTierNavigation") },
];

/** The capture-service switch: on/off follows the drone's reported readiness;
 * unknown (no readiness yet, or the node stopped answering) is disabled rather
 * than drawn as off. */
function CaptureServiceSwitch({ deviceId }: { deviceId: string | null }) {
  const control = useCaptureControl(deviceId);
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
    <div className="cap:flex cap:flex-col cap:items-end cap:gap-1">
      <button
        type="button"
        role="switch"
        aria-checked={checked}
        aria-label={t("featureLabel")}
        disabled={disabled}
        aria-busy={control.busy || undefined}
        onClick={() => void onToggle()}
        className={cn(
          "cap:relative cap:inline-flex cap:h-5 cap:w-9 cap:shrink-0 cap:items-center cap:rounded-full cap:transition-colors",
          "cap:focus-visible:outline-2 cap:focus-visible:outline-accent-primary",
          // A touch-sized hit area around the drawn switch under a coarse pointer.
          "cap:pointer-coarse:before:absolute cap:pointer-coarse:before:-inset-[14px] cap:pointer-coarse:before:content-['']",
          "cap:disabled:cursor-not-allowed cap:disabled:opacity-50",
          checked ? "cap:bg-accent-primary" : "cap:bg-bg-tertiary cap:border cap:border-border-default",
        )}
      >
        <span
          className={cn(
            "cap:inline-block cap:h-3.5 cap:w-3.5 cap:rounded-full cap:bg-text-primary cap:transition-transform",
            checked ? "cap:translate-x-[18px]" : "cap:translate-x-[3px]",
          )}
        />
      </button>
      {!known && <span className="cap:text-[10px] cap:text-text-tertiary">{tSettings("notReported")}</span>}
      {control.configError && (
        <p role="alert" className="cap:max-w-56 cap:text-right cap:text-[10px] cap:text-status-error">
          {control.configError}
        </p>
      )}
    </div>
  );
}

export function CaptureSetupPage() {
  const deviceId = useHost().node.deviceId;
  const { config, setValue, error } = usePluginConfig(deviceId);
  const readOnly = config === null;

  return (
    <div className="cap:min-h-0 cap:flex-1 cap:overflow-auto cap:p-4">
      <Section title={t("title")} icon={Boxes} blurb={t("blurb")}>
        <div className="cap:flex cap:items-start cap:justify-between cap:gap-3">
          <div className="cap:min-w-0">
            <div className="cap:text-xs cap:font-medium cap:text-text-primary">{t("featureLabel")}</div>
            <p className="cap:mt-0.5 cap:text-[11px] cap:text-text-tertiary">{t("featureHint")}</p>
          </div>
          <div className="cap:shrink-0 cap:pt-0.5">
            <CaptureServiceSwitch deviceId={deviceId} />
          </div>
        </div>

        {error && <InfoNote>{error}</InfoNote>}

        <div className="cap:border-t cap:border-border-default cap:pt-3">
          <ConfigSelectField
            configKey="capture.pose_source"
            label={t("poseTierLabel")}
            hint={t("poseTierHint")}
            options={POSE_SOURCE_OPTIONS}
            config={config}
            readOnly={readOnly}
            setValue={setValue}
          />
        </div>

        <div className="cap:space-y-2 cap:border-t cap:border-border-default cap:pt-3">
          <ConfigReadonlyRow
            configKey="capture.capture_profile"
            label={t("captureProfileLabel")}
            config={config}
            format={(raw) =>
              typeof raw === "string" && raw.length > 0 ? (CAPTURE_PROFILE_LABELS[raw] ?? raw) : null
            }
          />
          <ConfigReadonlyRow configKey="capture.reconstruct_steps" label={t("reconstructStepsLabel")} config={config} />
          <p className="cap:text-[11px] cap:text-text-tertiary">{t("captureManagedHint")}</p>
        </div>
      </Section>
    </div>
  );
}
