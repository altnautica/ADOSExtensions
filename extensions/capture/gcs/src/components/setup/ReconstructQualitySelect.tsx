/**
 * The reconstruction detail-level picker for a drone's capture. The
 * operator chooses how long the compute node trains a reconstruction (Draft to
 * Maximum, mapped to Brush training steps). The choice persists on the drone's
 * capture config (`capture.reconstruct_steps`) and rides the reconstruct job's
 * `params.steps`.
 */

import { Select } from "@altnautica/mapping-ui/components/ui/Select";
import { translator } from "../../host/i18n";
import type { CaptureControl } from "../../hooks/use-capture-control";
import {
  DEFAULT_RECONSTRUCTION_STEPS,
  RECONSTRUCTION_QUALITIES,
  qualityForSteps,
  stepsForQuality,
} from "@altnautica/mapping-ui/lib/reconstruction-quality";

const t = translator("setup");

const OPTIONS = RECONSTRUCTION_QUALITIES.map((q) => ({
  value: q.id,
  label: `${t(q.labelKey)}: ${t(q.descKey)}`,
}));

export function ReconstructQualitySelect({ control }: { control: CaptureControl }) {
  const steps = control.readiness?.reconstructSteps ?? DEFAULT_RECONSTRUCTION_STEPS;
  return (
    <div className="cap:flex cap:items-center cap:justify-between cap:gap-2 cap:text-[11px]">
      <span className="cap:text-text-tertiary">{t("reconstructQuality.label")}</span>
      <div className="cap:w-56">
        <Select
          options={OPTIONS}
          value={qualityForSteps(steps).id}
          onChange={(v) => void control.setReconstructSteps(stepsForQuality(v))}
          disabled={!control.live || control.busy}
        />
      </div>
    </div>
  );
}
