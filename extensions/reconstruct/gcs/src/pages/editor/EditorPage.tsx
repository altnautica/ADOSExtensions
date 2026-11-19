/**
 * @module editor/EditorPage
 * @description The point-cloud editor: a finished product's cloud with its
 * layers on the left, the canvas in the middle and provenance, measurement
 * and export on the right. Selection and measurement run in the browser on
 * the shown points; every operation that rewrites points runs on the node as
 * an `edit` job that writes a new version, and undo opens the version before.
 */

import { useEffect, useMemo, useReducer, useState } from "react";
import { Boxes } from "lucide-react";

import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";
import { Select } from "@altnautica/mapping-ui/components/ui/Select";
import { ViewerLoading } from "@altnautica/mapping-ui/components/model/viewers/ViewerLoading";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";
import { LOCALE } from "@altnautica/mapping-ui/host/i18n";
import type { ComputeAgentClient, ComputeJob } from "@altnautica/mapping-ui/lib/agent/compute-client";

import { translator } from "../../host/i18n";
import { exactVolume, versionsOf, type EditOp } from "./editor-api";
import { countRegion, type EditorCloud } from "./editor-cloud";
import { editorReducer, initialState, nextPatchName, picksNeeded, type EditorState, type MeasureKind } from "./editor-state";
import { EditorCanvas } from "./EditorCanvas";
import { distance, polygonArea, volumeAbove } from "./geometry";
import { LayersPanel } from "./LayersPanel";
import { ProvenancePanel } from "./ProvenancePanel";
import { Toolbar } from "./Toolbar";
import { BASE_ARTIFACT, useEditorProduct, useLeavePrompt, useSingleWriter } from "./use-editor-product";

const t = translator("editor");

/** Grid cell of a volume measurement, metres (the node's default). */
const VOLUME_CELL_M = 0.25;

const MEASURE_KINDS: MeasureKind[] = ["distance", "area", "height", "volume"];

/** The interactive measurement, or null until it has what it needs. */
export function measureOnShown(cloud: EditorCloud, s: EditorState): string | null {
  const hidden = new Set(s.hidden);
  const [a, b] = s.picks;
  switch (s.measureKind) {
    case "distance":
      return a && b ? `${distance(a, b).toFixed(3)} m` : null;
    case "height":
      return a && b ? `${(b[2] - a[2]).toFixed(3)} m` : null;
    case "area":
      return s.selection ? `${polygonArea(s.selection).toFixed(2)} m²` : null;
    case "volume": {
      if (!s.selection || !a) return null;
      const v = volumeAbove(cloud.positions, (i) => !hidden.has(cloud.layers[i]!), s.selection, a[2], VOLUME_CELL_M);
      return `${v.volumeM3.toFixed(2)} m³`;
    }
  }
}

function hintFor(s: EditorState): string {
  if (s.mode === "measure") {
    if (s.measureKind === "area" || s.measureKind === "volume") {
      if (!s.selection) return t("hint.measureSelect");
    }
    return t(`hint.measure.${s.measureKind}`);
  }
  if (s.mode === "patch") return t(s.patch?.box ? "hint.patchPlace" : "hint.patchSource");
  return t(`hint.${s.mode}`);
}

function Editor({ client, job }: { client: ComputeAgentClient; job: ComputeJob }) {
  const session = job.datasetId ?? job.sessionId;
  const product = useEditorProduct(client, job.id);
  const otherTab = useSingleWriter(`${client.deviceId}:${job.id}`);
  const [s, dispatch] = useReducer(editorReducer, initialState);
  const [message, setMessage] = useState<string | null>(null);
  const [decimatePct, setDecimatePct] = useState<string | null>(null);
  const [exact, setExact] = useState<string | null>(null);
  /** The last export's file under the product job's `out/`. */
  const [exported, setExported] = useState<string | null>(null);
  useLeavePrompt(s.pending.length > 0);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") {
        dispatch({ type: "escape" });
        setDecimatePct(null);
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);
  useEffect(() => setExact(null), [s.selection, s.picks, s.measureKind, product.version]);

  const ready = product.state.kind === "ready" ? product.state : null;
  const hidden = useMemo(() => new Set(s.hidden), [s.hidden]);
  const region = useMemo(
    () => (ready && s.selection ? countRegion(ready.cloud, { polygon: s.selection, z: s.band }, hidden) : null),
    [ready, s.selection, s.band, hidden],
  );

  if (product.state.kind === "loading") return <ViewerLoading label={t("loading")} />;
  if (product.state.kind === "missing") {
    // The reconstruction's own cloud missing means the job made none.
    const none = product.version === BASE_ARTIFACT;
    return <CalmState icon={Boxes} message={t(none ? "noCloud" : "sourceMissing")} />;
  }
  if (product.state.kind === "failed" || !ready) return <CalmState icon={Boxes} message={t("loadFailed")} />;
  const { cloud, edits } = ready;
  const readOnly = otherTab || !session || product.running !== null;
  // The open version's chain, then any later versions the newest edit wrote.
  const versions = [...new Set([...versionsOf(edits), ...product.chain])];
  const at = versions.indexOf(product.version ?? edits.product);

  const queue = (op: EditOp) => dispatch({ type: "queue", op });
  const runOps = async (ops: EditOp[]) => {
    if (!session) return;
    setMessage(null);
    const err = await product.run(session, ops);
    if (err) return setMessage(err);
    if (ops.some((o) => o.op !== "export")) return dispatch({ type: "submitted" });
    const name = (product.version ?? edits.product).replace(/^.*\//, "").replace(/\.laz$/, "");
    const op = ops[0];
    setExported(`exports/${name}.${op?.op === "export" ? op.format : "laz"}`);
    setMessage(t("exported"));
  };
  const download = async (rel: string) => {
    const res = await client.fileSource(job.id, rel).fetch(AbortSignal.timeout(600_000));
    if (!res.ok) return setMessage(t("loadFailed"));
    const url = URL.createObjectURL(await res.blob());
    const a = document.createElement("a");
    a.href = url;
    a.download = rel.replace(/^.*\//, "");
    a.click();
    URL.revokeObjectURL(url);
  };
  const regionOf = () => (s.selection ? { polygon: s.selection, ...(s.band ? { z: s.band } : {}) } : null);
  const measurement = s.mode === "measure" || s.picks.length > 0 ? (exact ?? measureOnShown(cloud, s)) : null;

  return (
    <div className="rec:flex rec:h-full rec:min-h-[480px] rec:flex-col">
      <div className="rec:flex rec:flex-wrap rec:items-center rec:gap-2 rec:border-b rec:border-border-default rec:px-3 rec:py-2 rec:text-xs">
        <Select
          options={versions.map((v, i) => ({ value: v, label: i === 0 ? t("baseVersion") : v.replace(/^out\//, "") }))}
          value={product.version ?? edits.product}
          onChange={(v) => product.setVersion(v)}
          className="rec:w-56"
        />
        <span className="rec:tabular-nums rec:text-text-tertiary">
          {cloud.reduced
            ? t("shownOf", { shown: cloud.layers.length.toLocaleString(LOCALE), total: cloud.total.toLocaleString(LOCALE) })
            : t("points", { total: cloud.total.toLocaleString(LOCALE) })}
        </span>
        {edits.edited && <span className="rec:rounded rec:bg-status-warning/15 rec:px-1.5 rec:text-status-warning">{t("edited")}</span>}
        {otherTab && <span className="rec:text-status-warning">{t("otherTab")}</span>}
        {!session && <span className="rec:text-status-warning">{t("noSession")}</span>}
        {product.running && <span className="rec:text-text-secondary">{t("running")}</span>}
        {message && <span className="rec:text-status-warning" role="alert">{message}</span>}
        {exported && (
          <>
            <Button size="sm" variant="ghost" onClick={() => void download(exported)}>
              {t("download", { file: exported.replace(/^.*\//, "") })}
            </Button>
            <Button
              size="sm"
              variant="ghost"
              onClick={() => void download(exported.replace(/\.[^.]+$/, ".manifest.json"))}
            >
              {t("downloadManifest")}
            </Button>
          </>
        )}
      </div>
      <Toolbar
        mode={s.mode}
        view={s.view}
        readOnly={readOnly}
        hasSelection={!!s.selection}
        canUndo={at > 0 && s.pending.length === 0}
        hint={hintFor(s)}
        refusal={s.refusal}
        onMode={(mode) => dispatch({ type: "mode", mode })}
        onCrop={() => {
          const r = regionOf();
          if (r) queue({ op: "crop", keep: "inside", ...r });
        }}
        onDelete={() => {
          const r = regionOf();
          if (r) queue({ op: "delete", ...r });
        }}
        onDecimate={() => setDecimatePct("50")}
        onView={() => dispatch({ type: "view", view: s.view === "top" ? "3d" : "top" })}
        onUndo={() => product.setVersion(versions[at - 1]!)}
      />
      {(s.selection || s.mode === "box" || s.mode === "polygon") && (
        <div className="rec:flex rec:flex-wrap rec:items-center rec:gap-3 rec:border-b rec:border-border-default rec:px-3 rec:py-1 rec:text-xs">
          <span className="rec:tabular-nums" data-testid="selection-count">
            {region ? t("selected", { count: region.count.toLocaleString(LOCALE) }) : t("noSelection")}
          </span>
          <label className="rec:flex rec:items-center rec:gap-1">
            <input
              type="checkbox"
              checked={s.band !== null}
              onChange={(e) => dispatch({ type: "band", band: e.target.checked ? cloud.zRange : null })}
            />
            {t("heightBand")}
          </label>
          {s.band &&
            ([0, 1] as const).map((k) => (
              <input
                key={k}
                aria-label={k === 0 ? t("bandLow") : t("bandHigh")}
                type="number"
                step="0.1"
                value={s.band![k].toFixed(2)}
                onChange={(e) => {
                  const v = Number(e.target.value);
                  const band: [number, number] = [...s.band!] as [number, number];
                  band[k] = v;
                  dispatch({ type: "band", band });
                }}
                className="rec:w-20 rec:rounded rec:bg-bg-secondary rec:px-1 rec:tabular-nums"
              />
            ))}
          {s.selection && !readOnly && (
            <Button
              size="sm"
              variant="ghost"
              onClick={() => {
                const r = regionOf();
                if (r) queue({ op: "crop", keep: "outside", ...r });
              }}
            >
              {t("cropOutside")}
            </Button>
          )}
        </div>
      )}
      {s.mode === "patch" && s.patch?.box && (
        <div className="rec:flex rec:items-center rec:gap-3 rec:border-b rec:border-border-default rec:px-3 rec:py-1 rec:text-xs">
          <label className="rec:flex rec:items-center rec:gap-1">
            {t("rotate")}
            <input
              type="number"
              step="5"
              value={s.patch.rotateDeg}
              onChange={(e) => dispatch({ type: "patchRotate", deg: Number(e.target.value) })}
              className="rec:w-16 rec:rounded rec:bg-bg-secondary rec:px-1 rec:tabular-nums"
            />
          </label>
          <Button
            size="sm"
            onClick={() =>
              queue({
                op: "patch",
                layer: nextPatchName([
                  ...edits.layers.map((l) => l.name),
                  ...s.pending.flatMap((o) => (o.op === "patch" ? [o.layer] : [])),
                ]),
                source: { box: s.patch!.box! },
                target: { translate: s.patch!.translate, rotate_deg: s.patch!.rotateDeg },
              })
            }
          >
            {t("queuePatch")}
          </Button>
        </div>
      )}
      {decimatePct !== null && (
        <div className="rec:flex rec:items-center rec:gap-3 rec:border-b rec:border-border-default rec:px-3 rec:py-1 rec:text-xs">
          <label className="rec:flex rec:items-center rec:gap-1">
            {t("keepPercent")}
            <input
              inputMode="decimal"
              value={decimatePct}
              onChange={(e) => setDecimatePct(e.target.value.replace(/[^0-9.]/g, ""))}
              className="rec:w-14 rec:rounded rec:bg-bg-secondary rec:px-1 rec:text-right rec:tabular-nums"
            />
            %
          </label>
          <Button
            size="sm"
            disabled={!(Number(decimatePct) > 0 && Number(decimatePct) < 100)}
            onClick={() => {
              queue({ op: "decimate", target_percent: Number(decimatePct) });
              setDecimatePct(null);
            }}
          >
            {t("queueDecimate")}
          </Button>
        </div>
      )}
      {s.mode === "measure" && (
        <div className="rec:flex rec:items-center rec:gap-2 rec:border-b rec:border-border-default rec:px-3 rec:py-1 rec:text-xs">
          {MEASURE_KINDS.map((k) => (
            <button
              key={k}
              type="button"
              aria-pressed={s.measureKind === k}
              onClick={() => dispatch({ type: "measureKind", kind: k })}
              className={s.measureKind === k ? "rec:text-accent-primary" : "rec:text-text-secondary"}
            >
              {t(`measure.${k}`)}
            </button>
          ))}
          {s.measureKind === "volume" && s.selection && s.picks.length >= picksNeeded("volume") && (
            <Button
              size="sm"
              variant="ghost"
              onClick={async () => {
                const r = await exactVolume(client, {
                  job: job.id,
                  source: product.version ?? edits.product,
                  polygon: s.selection!,
                  base_z: s.picks[0]![2],
                  cell_m: VOLUME_CELL_M,
                });
                setExact(typeof r === "string" ? r : t("exactVolume", { v: r.toFixed(2) }));
              }}
            >
              {t("exact")}
            </Button>
          )}
        </div>
      )}
      <div className="rec:grid rec:min-h-0 rec:flex-1 rec:grid-cols-[14rem_1fr_17rem]">
        <LayersPanel
          edits={edits}
          hidden={s.hidden}
          readOnly={readOnly}
          onToggle={(id) => dispatch({ type: "toggleLayer", id })}
          onDelete={(layer, refusal) =>
            refusal ? dispatch({ type: "refuse", message: refusal }) : queue({ op: "delete_layer", layer })
          }
        />
        <div className="rec:relative rec:min-h-0 rec:border-x rec:border-border-default">
          <EditorCanvas cloud={cloud} state={s} dispatch={dispatch} />
        </div>
        <div className="rec:min-h-0 rec:overflow-y-auto">
          <ProvenancePanel
            edits={edits}
            pending={s.pending}
            measurement={measurement}
            client={client}
            job={job.id}
            readOnly={readOnly}
            busy={product.running !== null}
            onSubmit={() => void runOps(s.pending)}
            onExport={(op) => void runOps([op])}
          />
        </div>
      </div>
    </div>
  );
}

/** A finished product job that holds a cloud an editor can open. */
function isProduct(j: ComputeJob): boolean {
  return j.state === "completed" && j.kind !== "edit" && j.kind !== "perception_offload";
}

export function EditorPage({ deviceId }: { deviceId: string | null }) {
  const { jobs, client } = useComputeJobs(deviceId);
  const products = useMemo(() => jobs.filter(isProduct).sort((a, b) => b.createdMs - a.createdMs), [jobs]);
  const [picked, setPicked] = useState("");
  const job = products.find((j) => j.id === picked) ?? products[0];

  if (!client) return <CalmState icon={Boxes} message={t("localOnly")} />;
  if (!job) return <CalmState icon={Boxes} message={t("noJobs")} />;
  return (
    <div className="rec:flex rec:h-full rec:flex-col">
      <div className="rec:flex rec:items-center rec:gap-2 rec:border-b rec:border-border-default rec:p-3">
        <Select
          options={products.map((j) => ({ value: j.id, label: `${j.kind} · ${(j.sessionId ?? j.id).slice(0, 12)}` }))}
          value={job.id}
          onChange={setPicked}
          placeholder={t("pickJob")}
          className="rec:w-72"
        />
      </div>
      <div className="rec:min-h-0 rec:flex-1">
        <Editor key={job.id} client={client} job={job} />
      </div>
    </div>
  );
}
