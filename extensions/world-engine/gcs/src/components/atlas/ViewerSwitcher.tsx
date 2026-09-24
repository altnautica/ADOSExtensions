/**
 * @module atlas/ViewerSwitcher
 * @description The World Model viewer switcher toolbar — a small segmented
 * button row that flips the active {@link AtlasViewer} (World / Splat / Cloud /
 * LOD) over the same reconstruction. Extracted from the two original call sites
 * (the drone World Model tab and the workstation Forge Outputs view) so every
 * surface that renders a world model — a first-party feature, the Forge
 * workbench, or a future plugin viewer host — drives the same control.
 *
 * Controlled: the caller owns the selected `viewer` and its per-viewer artifact
 * resolution; this component only renders the buttons and reports a selection.
 *
 */

import { cn } from "../../lib/utils";
import {
  ATLAS_VIEWERS,
  type AtlasViewer,
  type AtlasViewerSpec,
} from "./viewer-types";

interface ViewerSwitcherProps {
  /** The currently-selected viewer (controlled). */
  viewer: AtlasViewer;
  /** Called with the picked viewer id. */
  onSelect: (viewer: AtlasViewer) => void;
  /** Accessible label for the button group. */
  ariaLabel: string;
  /** The viewers to offer; defaults to the full {@link ATLAS_VIEWERS} set. */
  viewers?: readonly AtlasViewerSpec[];
  /** Extra classes on the group wrapper (defaults to a right-aligned row). */
  className?: string;
}

export function ViewerSwitcher({
  viewer,
  onSelect,
  ariaLabel,
  viewers = ATLAS_VIEWERS,
  className,
}: ViewerSwitcherProps) {
  return (
    <div
      className={cn("we:flex we:items-center we:gap-1 we:ml-auto", className)}
      role="group"
      aria-label={ariaLabel}
    >
      {viewers.map((v) => (
        <button
          key={v.id}
          type="button"
          aria-pressed={viewer === v.id}
          onClick={() => onSelect(v.id)}
          className={cn(
            "we:text-[11px] we:px-2 we:py-1 we:rounded we:transition-colors",
            viewer === v.id
              ? "we:bg-accent-primary/20 we:text-accent-primary"
              : "we:text-text-tertiary we:hover:text-text-secondary",
          )}
        >
          {v.label}
        </button>
      ))}
    </div>
  );
}
