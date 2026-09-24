/**
 * @module SubmitJobButton
 * @description Queue a reconstruction from the workstation itself, over one of
 * the datasets this node already produced work for, so its own Jobs view can
 * start work (including re-running a dataset whose job failed) and not only
 * cancel it.
 */

import { useState } from "react";

import { Button } from "../../components/ui/Button";
import { Select } from "../../components/ui/Select";
import { useToast } from "../../components/ui/Toast";
import { translator } from "../../host/i18n";
import type { ComputeAgentClient, ComputeJob } from "../../lib/agent/compute-client";

const t = translator("atlas");

/** The reconstructor's shipped default training-step count. */
const DEFAULT_RECONSTRUCT_STEPS = 30000;

/** The params that attribute a reconstruction to its drone and session; the
 * worker publishes a world model only with a device_id. */
const ATTRIBUTION_KEYS = ["device_id", "session_id", "generation", "backend"] as const;

export function SubmitJobButton({
  client,
  datasetIds,
  jobs,
}: {
  client: ComputeAgentClient;
  datasetIds: readonly string[];
  /** Jobs already on this node; a reconstruct job on the same dataset carries
   * the drone and session the dataset came from. */
  jobs: readonly ComputeJob[];
}) {
  const { toast } = useToast();
  const [open, setOpen] = useState(false);
  const [datasetId, setDatasetId] = useState("");
  const [steps, setSteps] = useState(String(DEFAULT_RECONSTRUCT_STEPS));
  const [busy, setBusy] = useState(false);

  const effectiveDataset = datasetId || (datasetIds[0] ?? "");

  const submit = async () => {
    if (!effectiveDataset) return;
    setBusy(true);
    const parsed = Number.parseInt(steps, 10);
    const sibling = jobs.find((j) => j.datasetId === effectiveDataset && j.kind === "reconstruct");
    const attribution: Record<string, unknown> = {};
    for (const k of ATTRIBUTION_KEYS) {
      if (sibling?.params[k] !== undefined) attribution[k] = sibling.params[k];
    }
    const result = await client.submitJob({
      kind: "reconstruct",
      datasetId: effectiveDataset,
      params: { ...attribution, steps: Number.isFinite(parsed) ? parsed : DEFAULT_RECONSTRUCT_STEPS },
    });
    setBusy(false);
    if (result === null) {
      toast(t("forgeSubmitFailed"), "error");
      return;
    }
    toast(t("forgeSubmitQueued"), "success");
    setOpen(false);
  };

  if (datasetIds.length === 0) {
    // Nothing to reconstruct from: say why the control is absent rather than
    // offer one that can only fail.
    return <span className="we:text-[11px] we:text-text-tertiary">{t("forgeNoDatasets")}</span>;
  }

  if (!open) {
    return (
      <Button size="sm" variant="secondary" onClick={() => setOpen(true)}>
        {t("forgeSubmit")}
      </Button>
    );
  }

  return (
    <div className="we:flex we:items-center we:gap-2">
      <Select
        options={datasetIds.map((id) => ({ value: id, label: id }))}
        value={effectiveDataset}
        onChange={setDatasetId}
        className="we:w-44"
      />
      <input
        type="number"
        min={1000}
        step={1000}
        value={steps}
        onChange={(e) => setSteps(e.target.value)}
        aria-label={t("forgeSteps")}
        className="we:h-8 we:w-24 we:rounded we:border we:border-border-default we:bg-bg-tertiary we:px-2 we:py-1 we:text-xs we:text-text-primary"
      />
      <Button size="sm" onClick={() => void submit()} disabled={busy} loading={busy}>
        {t("forgeSubmit")}
      </Button>
      <Button size="sm" variant="ghost" onClick={() => setOpen(false)}>
        {t("forgeSubmitCancel")}
      </Button>
    </div>
  );
}
