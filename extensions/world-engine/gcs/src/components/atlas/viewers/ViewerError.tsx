/**
 * @module atlas/viewers/ViewerError
 * @description The honest failure overlay for a World Model viewer — shown when
 * the viewer's code chunk or its remote artifact fails to load, so the operator
 * sees "failed to load" rather than a permanently-blank viewport (no fabricated reading).
 */

import { AlertTriangle } from "lucide-react";
import { translator } from "../../../host/i18n";

const t = translator("atlas");

export function ViewerError({ what }: { what: string }) {
  return (
    <div className="we:absolute we:inset-0 we:flex we:items-center we:justify-center we:bg-bg-primary/60 we:p-6">
      <div className="we:flex we:items-center we:gap-2 we:text-[11px] we:text-status-warning">
        <AlertTriangle className="we:w-4 we:h-4" />
        <span>{t("viewerError", { what })}</span>
      </div>
    </div>
  );
}
