/**
 * @module viewers/ViewerError
 * @description The honest failure overlay for a model viewer — shown when
 * the viewer's code chunk or its remote artifact fails to load, or the model
 * loaded but holds nothing to see, so the operator sees why rather than a
 * permanently-blank viewport (no fabricated reading).
 */

import { AlertTriangle } from "lucide-react";
import { viewerT as t } from "../../../host/i18n";


/** `what` names the model that failed to load; `message` replaces the text. */
export function ViewerError(props: { what: string } | { message: string }) {
  return (
    <div className="map:absolute map:inset-0 map:flex map:items-center map:justify-center map:bg-bg-primary/60 map:p-6">
      <div className="map:flex map:items-center map:gap-2 map:text-[11px] map:text-status-warning">
        <AlertTriangle className="map:w-4 map:h-4" />
        <span>{"message" in props ? props.message : t("viewerError", { what: props.what })}</span>
      </div>
    </div>
  );
}
