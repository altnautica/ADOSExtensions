/**
 * @module editor/editor-state
 * @description The editor's interaction state: the active tool, the view,
 * the selection and its height band, a patch being placed, the operations
 * queued for the next edit job, hidden layers and the current measurement.
 * Escape always returns to Select. Selection is drawn in top view, where the
 * screen is the ground plane, so what is selected is exactly the footprint
 * the node applies.
 */

import type { EditOp } from "./editor-api";
import { MAX_POLYGON_VERTICES, selfIntersects, type Xy, type Xyz } from "./geometry";

export type Mode = "select" | "box" | "polygon" | "patch" | "measure";
export type MeasureKind = "distance" | "area" | "height" | "volume";

export interface Patch {
  box: [Xy, Xy] | null;
  translate: Xy;
  rotateDeg: number;
}

export interface EditorState {
  mode: Mode;
  view: "top" | "3d";
  /** The committed selection polygon (ground plane). */
  selection: Xy[] | null;
  /** Vertices of a polygon being drawn. */
  draft: Xy[];
  /** Height band of the selection; null = every height. */
  band: [number, number] | null;
  patch: Patch | null;
  pending: EditOp[];
  hidden: number[];
  measureKind: MeasureKind;
  picks: Xyz[];
  /** One sentence for the hint line when an action was refused. */
  refusal: string | null;
}

export const initialState: EditorState = {
  mode: "select",
  view: "top",
  selection: null,
  draft: [],
  band: null,
  patch: null,
  pending: [],
  hidden: [],
  measureKind: "distance",
  picks: [],
  refusal: null,
};

export type Action =
  | { type: "mode"; mode: Mode }
  | { type: "escape" }
  | { type: "view"; view: "top" | "3d" }
  | { type: "select"; polygon: Xy[] }
  | { type: "vertex"; at: Xy }
  | { type: "close" }
  | { type: "band"; band: [number, number] | null }
  | { type: "patchBox"; box: [Xy, Xy] }
  | { type: "patchPlace"; translate: Xy }
  | { type: "patchRotate"; deg: number }
  | { type: "queue"; op: EditOp }
  | { type: "unqueue" }
  | { type: "submitted" }
  | { type: "toggleLayer"; id: number }
  | { type: "measureKind"; kind: MeasureKind }
  | { type: "pick"; at: Xyz }
  | { type: "refuse"; message: string };

export const SELF_INTERSECTING = "The selection polygon crosses itself; draw it without crossing lines.";
export const TOO_MANY_CORNERS = "A selection polygon may have at most 256 corners.";

/** Points a measurement picks before it reads: two ends of a distance, a
 * base then a point for a height, the base plane of a volume (over the
 * selection), none for an area (the selection itself). */
export function picksNeeded(kind: MeasureKind): number {
  if (kind === "distance" || kind === "height") return 2;
  return kind === "volume" ? 1 : 0;
}

export function editorReducer(s: EditorState, a: Action): EditorState {
  switch (a.type) {
    case "mode":
      // Selection and patching are drawn on the ground plane.
      return {
        ...s,
        mode: a.mode,
        view: a.mode === "box" || a.mode === "polygon" || a.mode === "patch" ? "top" : s.view,
        draft: [],
        picks: [],
        patch: a.mode === "patch" ? { box: null, translate: [0, 0], rotateDeg: 0 } : null,
        refusal: null,
      };
    case "escape":
      return { ...s, mode: "select", draft: [], picks: [], patch: null, refusal: null };
    case "view":
      return { ...s, view: a.view };
    case "select":
      if (selfIntersects(a.polygon)) return { ...s, refusal: SELF_INTERSECTING };
      return { ...s, selection: a.polygon, draft: [], refusal: null };
    case "vertex":
      if (s.draft.length >= MAX_POLYGON_VERTICES) return { ...s, refusal: TOO_MANY_CORNERS };
      return { ...s, draft: [...s.draft, a.at], refusal: null };
    case "close":
      if (s.draft.length < 3) return s;
      if (selfIntersects(s.draft)) return { ...s, draft: [], refusal: SELF_INTERSECTING };
      return { ...s, selection: s.draft, draft: [], refusal: null };
    case "band":
      return { ...s, band: a.band };
    case "patchBox":
      return { ...s, patch: { box: a.box, translate: [0, 0], rotateDeg: 0 } };
    case "patchPlace":
      return s.patch?.box ? { ...s, patch: { ...s.patch, translate: a.translate } } : s;
    case "patchRotate":
      return s.patch ? { ...s, patch: { ...s.patch, rotateDeg: a.deg } } : s;
    case "queue":
      return { ...s, pending: [...s.pending, a.op], selection: null, patch: null, mode: "select", refusal: null };
    case "unqueue":
      return { ...s, pending: s.pending.slice(0, -1) };
    case "submitted":
      return { ...s, pending: [] };
    case "toggleLayer":
      return {
        ...s,
        hidden: s.hidden.includes(a.id) ? s.hidden.filter((h) => h !== a.id) : [...s.hidden, a.id],
      };
    case "measureKind":
      return { ...s, measureKind: a.kind, picks: [], draft: [] };
    case "pick": {
      const need = picksNeeded(s.measureKind);
      if (need === 0) return s;
      const picks = s.picks.length >= need ? [a.at] : [...s.picks, a.at];
      return { ...s, picks };
    }
    case "refuse":
      return { ...s, refusal: a.message };
  }
}

/** The next free patch layer name. */
export function nextPatchName(taken: readonly string[]): string {
  for (let i = 1; ; i++) {
    const name = `patch-${i}`;
    if (!taken.includes(name)) return name;
  }
}
