/**
 * @module editor/LayersPanel
 * @description The left column: the base layer and every patch layer of the
 * open version, each with its point count, a visibility toggle and a delete.
 * A layer another patch was made from cannot be deleted; the refusal names
 * the dependent layer.
 */

import { Eye, EyeOff, Trash2 } from "lucide-react";

import { LOCALE } from "@altnautica/mapping-ui/host/i18n";

import { translator } from "../../host/i18n";
import { BASE_LAYER, type EditsFile } from "./editor-api";

const t = translator("editor");

/** The live patch layer made from `layer`, if any. */
export function dependentOf(edits: EditsFile, layer: string): string | null {
  const live = new Set(edits.layers.map((l) => l.name));
  for (const r of edits.ops) {
    if (r.op === "patch" && live.has(r.layer) && (r.source_layers ?? []).includes(layer)) return r.layer;
  }
  return null;
}

export function LayersPanel({
  edits,
  hidden,
  readOnly,
  onToggle,
  onDelete,
}: {
  edits: EditsFile;
  hidden: readonly number[];
  readOnly: boolean;
  onToggle: (id: number) => void;
  onDelete: (layer: string, refusal: string | null) => void;
}) {
  return (
    <section aria-label={t("layers")} className="rec:flex rec:flex-col rec:gap-1 rec:p-3">
      <h3 className="rec:text-xs rec:font-medium rec:text-text-secondary">{t("layers")}</h3>
      <ul className="rec:flex rec:flex-col rec:gap-1">
        {edits.layers.map((l) => {
          const shown = !hidden.includes(l.id);
          const dep = dependentOf(edits, l.name);
          return (
            <li key={l.id} className="rec:flex rec:items-center rec:gap-2 rec:rounded rec:px-2 rec:py-1 rec:text-xs rec:bg-bg-secondary">
              <button
                type="button"
                aria-pressed={shown}
                aria-label={shown ? t("hideLayer", { layer: l.name }) : t("showLayer", { layer: l.name })}
                onClick={() => onToggle(l.id)}
                className="rec:text-text-tertiary rec:hover:text-text-primary"
              >
                {shown ? <Eye size={13} aria-hidden /> : <EyeOff size={13} aria-hidden />}
              </button>
              <span className="rec:flex-1 rec:truncate rec:text-text-primary">{l.name}</span>
              <span className="rec:font-mono rec:tabular-nums rec:text-text-tertiary">{l.points.toLocaleString(LOCALE)}</span>
              {l.name !== BASE_LAYER && !readOnly && (
                <button
                  type="button"
                  aria-label={t("deleteLayer", { layer: l.name })}
                  onClick={() => onDelete(l.name, dep ? t("layerReferenced", { dependent: dep }) : null)}
                  className="rec:text-text-tertiary rec:hover:text-status-error"
                >
                  <Trash2 size={13} aria-hidden />
                </button>
              )}
            </li>
          );
        })}
      </ul>
    </section>
  );
}
