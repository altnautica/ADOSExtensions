/**
 * @module atlas/ReconstructionBadge
 * @description A chip overlaid on a World Model viewer naming the backend
 * that reconstructed it (`brush` / `gsplat` / `msplat` / `colmap`), so an
 * operator can confirm which reconstructor ran. An unknown/absent backend (a
 * cloud world before the producer forwards the field) shows nothing.
 */

import { translator } from "../../host/i18n";

const t = translator("atlas");

/**
 * Absolutely positioned bottom-right and `pointer-events-none` so it never
 * intercepts viewer input or covers the viewer's toolbar; every World Model
 * render site wraps the viewport in a `relative` container.
 */
export function ReconstructionBadge({ backend }: { backend: string | null | undefined }) {
  const name = typeof backend === "string" ? backend.trim() : "";
  if (!name) return null;
  return (
    <div className="we:pointer-events-none we:absolute we:right-2 we:bottom-2 we:z-10">
      <span
        className="we:inline-flex we:items-center we:rounded we:px-1.5 we:py-0.5 we:text-[10px] we:font-medium we:text-text-tertiary we:bg-bg-tertiary we:ring-1 we:ring-border-default"
        title={t("reconstructedWithHint", { backend: name })}
      >
        {t("reconstructedWith", { backend: name })}
      </span>
    </div>
  );
}
