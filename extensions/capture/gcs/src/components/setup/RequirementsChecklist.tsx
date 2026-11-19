/**
 * The capture capture requirements list (cameras / compute node reachable /
 * capture service), one row per requirement with a check, warning, cross or
 * unknown icon and a "what to do" detail line. Pure presentation over the gate
 * `computeCaptureGate` returns: never a false green.
 */

import { AlertTriangle, Check, CircleHelp, X } from "lucide-react";

import { translator } from "../../host/i18n";
import type { CaptureRequirement, RequirementTone } from "../../lib/capture/requirements";
import { cn } from "@altnautica/mapping-ui/lib/utils";

const t = translator("setup");

const TONE_ICON: Record<RequirementTone, typeof Check> = {
  met: Check,
  warning: AlertTriangle,
  unmet: X,
  unknown: CircleHelp,
};

const TONE_COLOR: Record<RequirementTone, string> = {
  met: "cap:text-status-success",
  warning: "cap:text-status-warning",
  unmet: "cap:text-status-error",
  unknown: "cap:text-text-tertiary",
};

export function RequirementsChecklist({ requirements }: { requirements: readonly CaptureRequirement[] }) {
  return (
    <div className="cap:space-y-1.5">
      <h3 className="cap:text-xs cap:font-semibold cap:uppercase cap:tracking-wider cap:text-text-secondary">
        {t("capture.requirementsTitle")}
      </h3>
      <ul className="cap:space-y-1.5">
        {requirements.map((req) => {
          const Icon = TONE_ICON[req.tone];
          return (
            <li key={req.id} className="cap:flex cap:items-start cap:gap-1.5 cap:text-[11px]">
              <Icon size={12} className={cn("cap:mt-0.5 cap:shrink-0", TONE_COLOR[req.tone])} />
              <div className="cap:min-w-0 cap:flex-1">
                <span className="cap:font-medium cap:text-text-primary">{t(req.labelKey)}</span>
                <p className="cap:text-[10px] cap:text-text-tertiary">{t(req.detailKey, req.detailValues)}</p>
              </div>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
