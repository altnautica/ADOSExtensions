/**
 * @module model/ReconstructionBadge
 * @description A chip overlaid on a model viewer naming the backend
 * that reconstructed it (`brush` / `gsplat` / `msplat` / `colmap`), so an
 * operator can confirm which reconstructor ran. An unknown/absent backend (a
 * cloud world before the producer forwards the field) shows nothing.
 */

import { viewerT as t } from "../../host/i18n";


/**
 * Absolutely positioned bottom-right and `pointer-events-none` so it never
 * intercepts viewer input or covers the viewer's toolbar; every model
 * render site wraps the viewport in a `relative` container.
 */
export function ReconstructionBadge({ backend }: { backend: string | null | undefined }) {
  const name = typeof backend === "string" ? backend.trim() : "";
  if (!name) return null;
  return (
    <div className="map:pointer-events-none map:absolute map:right-2 map:bottom-2 map:z-10">
      <span
        className="map:inline-flex map:items-center map:rounded map:px-1.5 map:py-0.5 map:text-[10px] map:font-medium map:text-text-tertiary map:bg-bg-tertiary map:ring-1 map:ring-border-default"
        title={t("reconstructedWithHint", { backend: name })}
      >
        {t("reconstructedWith", { backend: name })}
      </span>
    </div>
  );
}
