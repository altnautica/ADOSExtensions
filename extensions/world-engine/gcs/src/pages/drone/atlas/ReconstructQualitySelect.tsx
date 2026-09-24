/**
 * The reconstruction detail-level picker for a drone's Atlas capture. The
 * operator chooses how long the compute node trains a reconstruction (Draft to
 * Maximum, mapped to Brush training steps). The choice persists on the drone's
 * atlas config (`reconstruct_steps`) and rides the reconstruct job's
 * `params.steps`.
 */

import { Select } from "../../../components/ui/Select";
import { translator } from "../../../host/i18n";
import type { AtlasControl } from "../../../hooks/use-atlas-control";
import {
  DEFAULT_RECONSTRUCTION_STEPS,
  RECONSTRUCTION_QUALITIES,
  qualityForSteps,
  stepsForQuality,
} from "../../../lib/atlas/reconstruction-quality";

const t = translator("atlas");

const OPTIONS = RECONSTRUCTION_QUALITIES.map((q) => ({
  value: q.id,
  label: `${t(q.labelKey)}: ${t(q.descKey)}`,
}));

export function ReconstructQualitySelect({ control }: { control: AtlasControl }) {
  const steps = control.readiness?.reconstructSteps ?? DEFAULT_RECONSTRUCTION_STEPS;
  return (
    <div className="we:flex we:items-center we:justify-between we:gap-2 we:text-[11px]">
      <span className="we:text-text-tertiary">{t("reconstructQuality.label")}</span>
      <div className="we:w-56">
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
