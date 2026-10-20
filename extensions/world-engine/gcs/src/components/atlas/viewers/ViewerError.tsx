/**
 * @module atlas/viewers/ViewerError
 * @description The honest failure overlay for a World Model viewer — shown when
 * the viewer's code chunk or its remote artifact fails to load, or the model
 * loaded but holds nothing to see, so the operator sees why rather than a
 * permanently-blank viewport (no fabricated reading).
 */

import { AlertTriangle } from "lucide-react";
import { translator } from "../../../host/i18n";

const t = translator("atlas");

/** `what` names the model that failed to load; `message` replaces the text. */
export function ViewerError(props: { what: string } | { message: string }) {
  return (
    <div className="we:absolute we:inset-0 we:flex we:items-center we:justify-center we:bg-bg-primary/60 we:p-6">
      <div className="we:flex we:items-center we:gap-2 we:text-[11px] we:text-status-warning">
        <AlertTriangle className="we:w-4 we:h-4" />
        <span>{"message" in props ? props.message : t("viewerError", { what: props.what })}</span>
      </div>
    </div>
  );
}
