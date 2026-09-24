/**
 * The Atlas capture requirements list (cameras / compute node reachable /
 * capture service), one row per requirement with a check, warning, cross or
 * unknown icon and a "what to do" detail line. Pure presentation over the gate
 * `computeCaptureGate` returns: never a false green.
 */

import { AlertTriangle, Check, CircleHelp, X } from "lucide-react";

import { translator } from "../../../host/i18n";
import type { AtlasRequirement, RequirementTone } from "../../../lib/atlas/capture-requirements";
import { cn } from "../../../lib/utils";

const t = translator("atlas");

const TONE_ICON: Record<RequirementTone, typeof Check> = {
  met: Check,
  warning: AlertTriangle,
  unmet: X,
  unknown: CircleHelp,
};

const TONE_COLOR: Record<RequirementTone, string> = {
  met: "we:text-status-success",
  warning: "we:text-status-warning",
  unmet: "we:text-status-error",
  unknown: "we:text-text-tertiary",
};

export function AtlasRequirementsChecklist({ requirements }: { requirements: readonly AtlasRequirement[] }) {
  return (
    <div className="we:space-y-1.5">
      <h3 className="we:text-xs we:font-semibold we:uppercase we:tracking-wider we:text-text-secondary">
        {t("capture.requirementsTitle")}
      </h3>
      <ul className="we:space-y-1.5">
        {requirements.map((req) => {
          const Icon = TONE_ICON[req.tone];
          return (
            <li key={req.id} className="we:flex we:items-start we:gap-1.5 we:text-[11px]">
              <Icon size={12} className={cn("we:mt-0.5 we:shrink-0", TONE_COLOR[req.tone])} />
              <div className="we:min-w-0 we:flex-1">
                <span className="we:font-medium we:text-text-primary">{t(req.labelKey)}</span>
                <p className="we:text-[10px] we:text-text-tertiary">{t(req.detailKey, req.detailValues)}</p>
              </div>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
