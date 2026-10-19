/**
 * @module calibration/CalibStepCard
 * @description One calibration wizard step: record the calibration session on
 * the device, upload it to the node, submit the node `calibrate` job for this
 * target, follow it, then review the report and push the resulting rig to the
 * device. The node refusing (a missing calibration tool) is shown as is.
 */

import { useEffect, useState } from "react";
import { Check, Circle, CloudUpload, Play, Square, Upload } from "lucide-react";

import { translator } from "../../host/i18n";
import { useComputeJobs } from "../../hooks/use-compute-jobs";
import type { CaptureClient } from "../../lib/agent/capture-client";
import type { ComputeAgentClient } from "../../lib/agent/compute-client";
import { coerceCalibReport, type CalibReport, type CalibStep } from "../../lib/capture/calibration";
import type { CaptureSession } from "../../lib/capture/sessions";
import { formatDuration } from "../../lib/capture/sessions";
import type { Rig } from "../../lib/contract/generated";
import { JobPipelineView } from "../sessions/JobPipelineView";
import { UploadCell } from "../sessions/SessionTable";
import { Button } from "../ui/Button";
import { useToast } from "../ui/Toast";

const t = translator("calibration");

export interface StepProgress {
  sessionId: string | null;
  recording: boolean;
  jobId: string | null;
}

/** The finished job's report and rig, read from its outputs. */
async function readResult(client: ComputeAgentClient, jobId: string): Promise<{ report: CalibReport | null; rig: Rig | null }> {
  const outs = (await client.getOutputs(jobId)) ?? [];
  const reportOut = outs.find((o) => o.kind === "calib_report");
  const rigOut = outs.find((o) => o.kind === "rig");
  const [report, rig] = await Promise.all([
    reportOut?.file ? client.readJsonFile(jobId, reportOut.file) : null,
    rigOut?.file ? client.readJsonFile(jobId, rigOut.file) : null,
  ]);
  return {
    report: coerceCalibReport(report),
    rig: rig && typeof rig === "object" && Array.isArray((rig as Rig).sensors) ? (rig as Rig) : null,
  };
}

export function CalibStepCard({
  index,
  step,
  progress,
  onProgress,
  device,
  session,
  nodeId,
  onApplied,
}: {
  index: number;
  step: CalibStep;
  progress: StepProgress;
  onProgress: (p: StepProgress) => void;
  device: CaptureClient;
  session: CaptureSession | null;
  nodeId: string | null;
  onApplied: () => void;
}) {
  const { toast } = useToast();
  const committed = session?.upload.state === "committed";
  const { jobs, client: nodeClient } = useComputeJobs(committed || progress.jobId ? nodeId : null);
  const job = jobs.find((j) => j.id === progress.jobId) ?? null;
  const [result, setResult] = useState<{ report: CalibReport | null; rig: Rig | null } | null>(null);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    if (!nodeClient || !job || job.state !== "completed") return;
    let cancelled = false;
    void readResult(nodeClient, job.id).then((r) => !cancelled && setResult(r));
    return () => {
      cancelled = true;
    };
  }, [nodeClient, job?.id, job?.state]);

  const act = async (fn: () => Promise<void>) => {
    setBusy(true);
    await fn();
    setBusy(false);
  };

  const record = () =>
    act(async () => {
      const res = await device.start({ site: "calibration", room: step.target });
      if (res.ok) onProgress({ sessionId: res.value.sessionId, recording: true, jobId: null });
      else toast(res.message, "error");
    });
  const stop = () =>
    act(async () => {
      const res = await device.stop();
      if (res.ok) onProgress({ ...progress, recording: false });
      else toast(res.message, "error");
    });
  const submit = (client: ComputeAgentClient, sessionId: string) =>
    act(async () => {
      const res = await client.submit("calibrate", { session_id: sessionId, target: step.target });
      if (res.ok) onProgress({ ...progress, jobId: res.jobId });
      else toast(res.code === "E_TOOL_MISSING" ? t("toolMissing", { detail: res.message }) : res.message, "error");
    });
  const apply = (rig: Rig) =>
    act(async () => {
      const res = await device.putRig(rig);
      if (res.ok) {
        toast(t("applied", { version: res.value.calibVersion ?? rig.calib_version }), "success");
        onApplied();
      } else toast(res.message, "error");
    });

  const done = job?.state === "completed";

  return (
    <li className="we:flex we:flex-col we:gap-2 we:border-t we:border-border-default we:pt-2" data-step={step.target}>
      <div className="we:flex we:items-center we:gap-2 we:text-xs">
        {done ? <Check size={12} className="we:text-status-success" /> : <Circle size={12} className="we:text-text-tertiary" />}
        <span className="we:font-medium we:text-text-primary">
          {index + 1}. {t(`step.${step.target}`)}
        </span>
        <span className="we:text-[11px] we:text-text-tertiary">{t("recordFor", { d: formatDuration(step.recordS) })}</span>
      </div>
      <p className="we:text-[11px] we:text-text-tertiary">{t(`hint.${step.target}`)}</p>
      <div className="we:flex we:flex-wrap we:items-center we:gap-2">
        {progress.recording ? (
          <Button size="sm" variant="danger" icon={<Square size={12} />} loading={busy} onClick={() => void stop()}>
            {t("stopRecording")}
          </Button>
        ) : (
          <Button size="sm" variant="secondary" icon={<Play size={12} />} loading={busy} onClick={() => void record()}>
            {progress.sessionId ? t("rerecord") : t("record")}
          </Button>
        )}
        {session && !progress.recording && <UploadCell session={session} rate={null} />}
        {session && !progress.recording && session.upload.state === "idle" && (
          <Button size="sm" variant="ghost" icon={<CloudUpload size={12} />} onClick={() => void device.upload(session.sessionId)}>
            {t("upload")}
          </Button>
        )}
        {committed && !progress.jobId && nodeClient && session && (
          <Button size="sm" icon={<Upload size={12} />} loading={busy} onClick={() => void submit(nodeClient, session.sessionId)}>
            {t("submit")}
          </Button>
        )}
        {committed && !nodeId && <span className="we:text-[11px] we:text-status-warning">{t("noNode")}</span>}
      </div>
      {job && <JobPipelineView job={job} />}
      {result?.report && (
        <dl className="we:grid we:grid-cols-[auto_1fr] we:gap-x-3 we:text-[11px]">
          {result.report.reprojectionRmsPx !== null && (
            <>
              <dt className="we:text-text-tertiary">{t("reprojection")}</dt>
              <dd className="we:font-mono">{result.report.reprojectionRmsPx.toFixed(3)} px</dd>
            </>
          )}
          {result.report.residuals.map((r) => (
            <div key={r.key} className="we:contents">
              <dt className="we:text-text-tertiary">{r.key}</dt>
              <dd className="we:font-mono">{r.value.toPrecision(4)}</dd>
            </div>
          ))}
          {result.report.tool && (
            <>
              <dt className="we:text-text-tertiary">{t("tool")}</dt>
              <dd className="we:font-mono">
                {result.report.tool} {result.report.toolVersion ?? ""}
              </dd>
            </>
          )}
        </dl>
      )}
      {result?.rig && (
        <Button size="sm" loading={busy} onClick={() => void apply(result.rig!)}>
          {t("apply", { version: result.rig.calib_version })}
        </Button>
      )}
    </li>
  );
}
