/**
 * @module editor/ProvenancePanel
 * @description The right column: the open version's operations in order
 * (what each removed or added, the version it wrote), the operations queued
 * for the next edit job, the current measurement, and the export controls
 * with the node's predicted size.
 */

import { useEffect, useState } from "react";

import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { LOCALE } from "@altnautica/mapping-ui/host/i18n";
import type { ComputeAgentClient } from "@altnautica/mapping-ui/lib/agent/compute-client";

import { translator } from "../../host/i18n";
import { estimateSize, type DecimateTarget, type EditOp, type EditRecord, type EditsFile } from "./editor-api";

const t = translator("editor");

const n = (v: number) => v.toLocaleString(LOCALE);

/** One sentence for an operation. */
export function describeOp(op: EditOp | EditRecord): string {
  const r = op as EditRecord;
  switch (op.op) {
    case "crop":
      return t(op.keep === "inside" ? "op.cropInside" : "op.cropOutside", { removed: n(r.removed ?? 0) });
    case "delete":
      return t("op.delete", { removed: n(r.removed ?? 0) });
    case "patch":
      return t(r.self_overlap ? "op.patchOverlap" : "op.patch", { layer: op.layer, added: n(r.added ?? 0) });
    case "delete_layer":
      return t("op.deleteLayer", { layer: op.layer, removed: n(r.removed ?? 0) });
    case "decimate":
      return r.deviation_m
        ? t("op.decimateDeviation", {
            removed: n(r.removed ?? 0),
            mean: r.deviation_m.mean.toFixed(3),
            p95: r.deviation_m.p95.toFixed(3),
          })
        : t("op.decimate", { removed: n(r.removed ?? 0) });
    case "export":
      return t("op.export", { format: op.format.toUpperCase() });
  }
}

/** One sentence for an operation queued for the next edit job (what it will
 * change is known only once the node ran it). */
export function describeQueued(op: EditOp): string {
  switch (op.op) {
    case "crop":
      return t(op.keep === "inside" ? "queued.cropInside" : "queued.cropOutside");
    case "delete":
      return t("queued.delete");
    case "patch":
      return t("queued.patch", { layer: op.layer });
    case "delete_layer":
      return t("queued.deleteLayer", { layer: op.layer });
    case "decimate":
      return t("queued.decimate", {
        target: op.target_points !== undefined ? n(op.target_points) : `${op.target_percent ?? 100} %`,
      });
    case "export":
      return t("queued.export", { format: op.format.toUpperCase() });
  }
}

/** A decimation percentage typed into the export form, when it is one. */
function decimateOf(percent: string): DecimateTarget | undefined {
  const pct = Number(percent);
  return percent && pct > 0 && pct < 100 ? { target_percent: pct } : undefined;
}

function formatBytes(b: number): string {
  if (b >= 1e9) return `${(b / 1e9).toFixed(2)} GB`;
  if (b >= 1e6) return `${(b / 1e6).toFixed(1)} MB`;
  return `${Math.max(1, Math.round(b / 1e3))} kB`;
}

export function ProvenancePanel({
  edits,
  pending,
  measurement,
  client,
  job,
  readOnly,
  busy,
  onSubmit,
  onExport,
}: {
  edits: EditsFile;
  pending: readonly EditOp[];
  measurement: string | null;
  client: ComputeAgentClient;
  job: string;
  readOnly: boolean;
  busy: boolean;
  onSubmit: () => void;
  onExport: (op: EditOp) => void;
}) {
  const [format, setFormat] = useState<"laz" | "ply">("laz");
  const [percent, setPercent] = useState("");
  const [ecef, setEcef] = useState(false);
  const [estimate, setEstimate] = useState<string | null>(null);
  const decimate = decimateOf(percent);

  useEffect(() => {
    let cancelled = false;
    setEstimate(null);
    const target = decimateOf(percent);
    void estimateSize(client, { job, source: edits.product, format, ...(target ? { decimate: target } : {}) }).then((r) => {
      if (cancelled) return;
      setEstimate(typeof r === "string" ? r : t("predicted", { size: formatBytes(r.bytes), points: n(r.points) }));
    });
    return () => {
      cancelled = true;
    };
  }, [client, job, edits.product, format, percent]);

  return (
    <aside className="rec:flex rec:flex-col rec:gap-4 rec:p-3 rec:text-xs" aria-label={t("provenance")}>
      <section>
        <h3 className="rec:mb-1 rec:font-medium rec:text-text-secondary">{t("provenance")}</h3>
        <p className="rec:mb-2 rec:text-text-tertiary">
          {edits.edited ? t("editedCount", { count: edits.ops.length }) : t("unedited")}
        </p>
        <ol className="rec:flex rec:flex-col rec:gap-1" data-testid="provenance-ops">
          {edits.ops.map((r, i) => (
            <li key={i} className="rec:flex rec:flex-col rec:rounded rec:bg-bg-secondary rec:px-2 rec:py-1">
              <span className="rec:text-text-primary">{describeOp(r)}</span>
              <span className="rec:font-mono rec:text-[10px] rec:text-text-tertiary">{r.result.replace(/^out\//, "")}</span>
            </li>
          ))}
        </ol>
      </section>

      {pending.length > 0 && (
        <section>
          <h3 className="rec:mb-1 rec:font-medium rec:text-text-secondary">{t("pending")}</h3>
          <ol className="rec:mb-2 rec:flex rec:flex-col rec:gap-1">
            {pending.map((op, i) => (
              <li key={i} className="rec:rounded rec:border rec:border-dashed rec:border-border-default rec:px-2 rec:py-1 rec:text-text-secondary">
                {describeQueued(op)}
              </li>
            ))}
          </ol>
          <Button size="sm" onClick={onSubmit} disabled={busy || readOnly}>
            {t("apply", { count: pending.length })}
          </Button>
        </section>
      )}

      {measurement && (
        <section>
          <h3 className="rec:mb-1 rec:font-medium rec:text-text-secondary">{t("measurement")}</h3>
          <p className="rec:font-mono rec:tabular-nums rec:text-text-primary" data-testid="measurement">
            {measurement}
          </p>
        </section>
      )}

      <section className="rec:flex rec:flex-col rec:gap-2">
        <h3 className="rec:font-medium rec:text-text-secondary">{t("export")}</h3>
        <label className="rec:flex rec:items-center rec:justify-between rec:gap-2">
          <span>{t("format")}</span>
          <select
            value={format}
            onChange={(e) => setFormat(e.target.value === "ply" ? "ply" : "laz")}
            className="rec:rounded rec:bg-bg-secondary rec:px-1 rec:py-0.5"
          >
            <option value="laz">LAZ</option>
            <option value="ply">PLY</option>
          </select>
        </label>
        <label className="rec:flex rec:items-center rec:justify-between rec:gap-2">
          <span>{t("decimateTo")}</span>
          <input
            inputMode="decimal"
            value={percent}
            onChange={(e) => setPercent(e.target.value.replace(/[^0-9.]/g, ""))}
            placeholder="100"
            className="rec:w-16 rec:rounded rec:bg-bg-secondary rec:px-1 rec:py-0.5 rec:text-right rec:tabular-nums"
          />
        </label>
        <label className="rec:flex rec:items-center rec:gap-2">
          <input type="checkbox" checked={ecef} onChange={(e) => setEcef(e.target.checked)} />
          <span>{t("ecef")}</span>
        </label>
        <p className="rec:text-text-tertiary">{edits.crs ? t("frame", { frame: edits.crs }) : t("siteFrame")}</p>
        <p className="rec:tabular-nums rec:text-text-secondary" data-testid="predicted-size">
          {estimate ?? t("predicting")}
        </p>
        <Button
          size="sm"
          variant="secondary"
          disabled={busy}
          onClick={() => onExport({ op: "export", format, ...(ecef ? { crs: "EPSG:4978" } : {}), ...(decimate ? { decimate } : {}) })}
        >
          {t("exportAction")}
        </Button>
      </section>
    </aside>
  );
}
