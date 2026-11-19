/**
 * @module model/ViewerSwitcher
 * @description The model viewer switcher toolbar — a small segmented
 * button row that flips the active {@link ModelViewer} (World / Splat / Cloud /
 * LOD) over the same reconstruction. Extracted from the two original call sites
 * (Capture's Model view and Reconstruct's Products) so every
 * surface that renders a reconstruction — a first-party feature, the products list
 * workbench, or a future plugin viewer host — drives the same control.
 *
 * Controlled: the caller owns the selected `viewer` and its per-viewer artifact
 * resolution; this component only renders the buttons and reports a selection.
 *
 */

import { viewerT as t } from "../../host/i18n";
import { cn } from "../../lib/utils";
import {
  MODEL_VIEWERS,
  type ModelViewer,
  type AtlasViewerSpec,
} from "./viewer-types";


interface ViewerSwitcherProps {
  /** The currently-selected viewer (controlled). */
  viewer: ModelViewer;
  /** Called with the picked viewer id. */
  onSelect: (viewer: ModelViewer) => void;
  /** Accessible label for the button group. */
  ariaLabel: string;
  /** The viewers to offer; defaults to the full {@link MODEL_VIEWERS} set. */
  viewers?: readonly AtlasViewerSpec[];
  /** Extra classes on the group wrapper (defaults to a right-aligned row). */
  className?: string;
}

export function ViewerSwitcher({
  viewer,
  onSelect,
  ariaLabel,
  viewers = MODEL_VIEWERS,
  className,
}: ViewerSwitcherProps) {
  return (
    <div
      className={cn("map:flex map:items-center map:gap-1 map:ml-auto", className)}
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
            "map:text-[11px] map:px-2 map:py-1 map:rounded map:transition-colors",
            "map:focus-visible:outline-2 map:focus-visible:outline-accent-primary",
            "map:pointer-coarse:min-h-12 map:pointer-coarse:min-w-12",
            viewer === v.id
              ? "map:bg-accent-primary/20 map:text-accent-primary"
              : "map:text-text-tertiary map:hover:text-text-secondary",
          )}
        >
          {t(`viewer.${v.id}`)}
        </button>
      ))}
    </div>
  );
}
