/**
 * @module atlas/ReconstructionBadge
 * @description The reconstruction-honesty badge overlaid on a World Model viewer
 * (no fabricated reading). A `mock` reconstruction is a deterministic placeholder produced on
 * a node with no GPU / no real backend installed — it is NEVER a real world
 * model, so it wears an unmissable warning chip. A real backend (`brush` /
 * `msplat` / `nerfstudio` / `colmap`) wears a calm neutral chip naming the
 * reconstructor, so an operator can confirm a genuine reconstruction ran. An
 * unknown/absent backend (a pre-field agent, or a cloud world before the
 * producer forwards the field) shows nothing.
 */

import { translator } from "../../host/i18n";

const t = translator("atlas");

/** Whether a backend name is the mock placeholder (case/whitespace-insensitive). */
export function isMockBackend(backend: string | null | undefined): boolean {
  return typeof backend === "string" && backend.trim().toLowerCase() === "mock";
}

/**
 * Overlaid honesty chip for a reconstruction's backend. Absolutely positioned
 * top-left and `pointer-events-none` so it never intercepts viewer input; every
 * World Model render site wraps the viewport in a `relative` container. Renders
 * nothing when the backend is unknown/absent.
 */
export function ReconstructionBadge({
  backend,
}: {
  backend: string | null | undefined;
}) {
  const name = typeof backend === "string" ? backend.trim() : "";
  if (!name) return null;
  const mock = isMockBackend(name);
  return (
    <div className="we:pointer-events-none we:absolute we:left-2 we:top-2 we:z-10">
      <span
        className={
          mock
            ? "we:inline-flex we:items-center we:rounded we:px-1.5 we:py-0.5 we:text-[10px] we:font-medium we:text-status-warning we:bg-status-warning/15 we:ring-1 we:ring-status-warning/30"
            : "we:inline-flex we:items-center we:rounded we:px-1.5 we:py-0.5 we:text-[10px] we:font-medium we:text-text-tertiary we:bg-bg-tertiary we:ring-1 we:ring-border-default"
        }
        title={
          mock
            ? t("placeholderArtifactHint")
            : t("reconstructedWithHint", { backend: name })
        }
      >
        {mock
          ? t("placeholderArtifactBadge")
          : t("reconstructedWith", { backend: name })}
      </span>
    </div>
  );
}
