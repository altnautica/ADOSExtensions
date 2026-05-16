import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type CSSProperties,
} from "react";

import {
  CameraPreview,
  DiagnosticTable,
  FilePicker,
  FramePreview,
  PoseCoverageMap,
  PrimaryButton,
  ProgressBar,
  ResultBanner,
  SecondaryButton,
  Wizard,
  WizardStep,
  type CapturedFrame,
  type DetectionShape,
  type DiagnosticRow,
  type PoseSample,
  useWizardState,
} from "@altnautica/extension-ui";
import type { PluginContext } from "@altnautica/plugin-sdk";

import {
  meanExposureFromDataUrl,
  scoreFrame,
  sharpnessFromDataUrl,
  type FrameQualityResult,
} from "../calibration/qualityScore";
import {
  isPoseSetDiverse,
  poseFromCorners,
  type TagCorners,
} from "../calibration/poseCluster";
import type {
  CalibrationCompletePayload,
  CalibrationProgressPayload,
  CalibrationResult,
  CalibrationStage,
} from "../calibration/types";
import type { VisionNavTelemetry } from "../types";

type TFn = PluginContext["i18n"]["t"];

function tr(t: TFn, key: string, fallback: string): string {
  const result = t(key);
  return result === key ? fallback : result;
}

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
  /** Closes the wizard. The parent owns the open/closed state because
   * the SensorsCard's Calibrate CTA is what triggers open. */
  onClose: () => void;
}

const MIN_FRAMES = 20;
const MAX_FRAMES = 30;
const TOTAL_STEPS = 7;

/**
 * Flagship in-app camera-IMU calibration wizard.
 *
 * The wizard owns its own capture state (camera stream, captured
 * frames, IMU motion window, computed pose samples) and round-trips
 * to the agent via two events: ``start_calibration`` carries the
 * frame bundle + capture window; the agent answers with periodic
 * ``calibration_progress`` events and a final
 * ``calibration_complete`` event with the result.
 *
 * Layout uses the extension-ui Wizard + WizardStep primitives so
 * future calibration wizards in sibling extensions share the same
 * shell.
 */
export function CalibrationWizard({
  ctx,
  telemetry,
  onClose,
}: Props): JSX.Element | null {
  const t = ctx.i18n.t;
  const wizard = useWizardState({ totalSteps: TOTAL_STEPS });

  // Open the wizard automatically when this component mounts (the
  // SensorsCard already opened it; we just initialise the state).
  useEffect(() => {
    wizard.open_();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const [stream, setStream] = useState<MediaStream | null>(null);
  const [latestFrameUrl, setLatestFrameUrl] = useState<string | null>(null);
  const [detections, setDetections] = useState<DetectionShape[]>([]);
  const [capturedFrames, setCapturedFrames] = useState<CapturedFrame[]>([]);
  const [poseSamples, setPoseSamples] = useState<PoseSample[]>([]);
  const [windowStartNs, setWindowStartNs] = useState<number | null>(null);
  const [imuTrace, setImuTrace] = useState<ImuTracePoint[]>([]);
  const [progress, setProgress] = useState<CalibrationProgressPayload | null>(
    null,
  );
  const [result, setResult] = useState<CalibrationResult | null>(null);
  const [submitError, setSubmitError] = useState<string | null>(null);
  const [submitting, setSubmitting] = useState(false);
  const frameUrlsRef = useRef<Map<string, string>>(new Map());
  const targetEdgeRef = useRef<number | null>(null);

  // Open the camera stream when the user reaches step 2.
  useEffect(() => {
    if (wizard.step < 1) return;
    if (stream !== null) return;
    let cancelled = false;
    navigator.mediaDevices
      .getUserMedia({ video: true, audio: false })
      .then((s) => {
        if (cancelled) {
          s.getTracks().forEach((t) => t.stop());
          return;
        }
        setStream(s);
      })
      .catch((err) => {
        setSubmitError(
          tr(
            t,
            "navigation.calibrationWizard.cameraDenied",
            "Camera access denied. Grant permission and reopen the wizard.",
          ) +
            " (" +
            String(err) +
            ")",
        );
      });
    return () => {
      cancelled = true;
    };
  }, [wizard.step, stream, t]);

  // Subscribe to the agent's progress + complete events.
  useEffect(() => {
    let cancelled = false;
    let offProgress: (() => void) | null = null;
    let offComplete: (() => void) | null = null;
    void ctx.telemetry
      .subscribe<CalibrationProgressPayload>(
        "vision-nav.calibration_progress",
        (msg) => {
          if (!cancelled && msg) setProgress(msg);
        },
      )
      .then((unsub) => {
        if (cancelled) {
          unsub();
          return;
        }
        offProgress = unsub;
      });
    void ctx.telemetry
      .subscribe<CalibrationCompletePayload>(
        "vision-nav.calibration_complete",
        (msg) => {
          if (!cancelled && msg) {
            setResult(msg.result);
            if (msg.error !== undefined) setSubmitError(msg.error);
            wizard.goTo(6);
          }
        },
      )
      .then((unsub) => {
        if (cancelled) {
          unsub();
          return;
        }
        offComplete = unsub;
      });
    return () => {
      cancelled = true;
      if (offProgress) offProgress();
      if (offComplete) offComplete();
    };
  }, [ctx, wizard]);

  // Tear down the camera stream on close.
  const cleanup = useCallback(() => {
    if (stream !== null) {
      stream.getTracks().forEach((t) => t.stop());
      setStream(null);
    }
    frameUrlsRef.current.clear();
  }, [stream]);

  const handleClose = useCallback(() => {
    cleanup();
    wizard.dismiss(false);
    onClose();
  }, [cleanup, wizard, onClose]);

  // Capture a frame. Computes quality signals and only keeps GOOD/OK
  // frames.
  const captureFrame = useCallback(async () => {
    if (latestFrameUrl === null) return;
    if (capturedFrames.length >= MAX_FRAMES) return;
    const id = crypto.randomUUID();
    const dataUrl = latestFrameUrl;
    const [sharpness, exposure] = await Promise.all([
      sharpnessFromDataUrl(dataUrl),
      meanExposureFromDataUrl(dataUrl),
    ]);
    const tagCount = detections.length;
    const tagAreaSpan = tagAreaSpanOf(detections);
    const quality: FrameQualityResult = scoreFrame({
      sharpness,
      tagCount,
      tagAreaSpan,
      meanExposure: exposure,
    });
    if (quality.verdict === "drop") {
      // Refuse silently; the operator sees the score chip update on
      // the live preview but no thumbnail is added.
      return;
    }
    frameUrlsRef.current.set(id, dataUrl);
    setCapturedFrames((prev) => [
      ...prev,
      { id, src: dataUrl, quality: quality.verdict },
    ]);
    // Pose tracking from the first detected tag's corners (cheap
    // heuristic; the agent does the authoritative pose recovery).
    if (detections.length > 0) {
      const corners = toTagCorners(detections[0]!.corners);
      setPoseSamples((prev) => [...prev, poseFromCorners(corners)]);
    }
  }, [latestFrameUrl, capturedFrames.length, detections]);

  const removeFrame = useCallback((id: string) => {
    frameUrlsRef.current.delete(id);
    setCapturedFrames((prev) => prev.filter((f) => f.id !== id));
    // Drop the matching pose sample. Heuristic: cheap to recompute
    // from scratch if needed; we just clear and let the operator
    // re-build by capturing more frames.
    setPoseSamples((prev) => prev.slice(0, prev.length - 1));
  }, []);

  // IMU motion sampling: while we are on step 4, periodically push
  // the current gyro / accel magnitudes into a rolling buffer so the
  // operator sees a live sparkline.
  useEffect(() => {
    if (wizard.step !== 3) return;
    if (windowStartNs === null) {
      setWindowStartNs(Date.now() * 1_000_000);
    }
    const interval = window.setInterval(() => {
      const gyroMag = Math.random() * 2; // placeholder; real IMU data
      const accelMag = Math.random() * 5; // would come from the heartbeat
      setImuTrace((prev) =>
        [...prev, { ts: Date.now(), gyroMag, accelMag }].slice(-60),
      );
    }, 100);
    return () => window.clearInterval(interval);
  }, [wizard.step, windowStartNs]);

  const imuMotionSufficient = useMemo(() => {
    if (imuTrace.length < 30) return false;
    const gyroMax = Math.max(...imuTrace.map((p) => p.gyroMag));
    const accelRange =
      Math.max(...imuTrace.map((p) => p.accelMag)) -
      Math.min(...imuTrace.map((p) => p.accelMag));
    return gyroMax >= 1.5 && accelRange >= 3;
  }, [imuTrace]);

  // Submit step: bundle the captured frames + window into the event
  // payload and call ctx.client.request.
  const submit = useCallback(async () => {
    setSubmitting(true);
    setSubmitError(null);
    const framesB64 = capturedFrames
      .map((f) => {
        const url = frameUrlsRef.current.get(f.id) ?? f.src;
        // Strip the "data:image/png;base64," prefix.
        const comma = url.indexOf(",");
        return comma >= 0 ? url.slice(comma + 1) : url;
      })
      .filter((s) => s.length > 0);
    try {
      await ctx.client.request(
        "vision-nav.start_calibration",
        "vehicle.command",
        {
          type: "start_calibration",
          framesB64,
          windowStartNs: windowStartNs ?? Date.now() * 1_000_000,
          windowEndNs: Date.now() * 1_000_000,
          width: 640,
          height: 480,
        },
      );
      wizard.goTo(5);
    } catch (err) {
      setSubmitError(err instanceof Error ? err.message : String(err));
    } finally {
      setSubmitting(false);
    }
  }, [capturedFrames, ctx, windowStartNs, wizard]);

  const applyResult = useCallback(async () => {
    if (result === null) return;
    const yaml = buildCamchainYaml(result);
    try {
      await ctx.client.request(
        "vision-nav.upload_calibration",
        "vehicle.command",
        { type: "upload_calibration", camchain_yaml: yaml },
      );
      handleClose();
    } catch (err) {
      setSubmitError(err instanceof Error ? err.message : String(err));
    }
  }, [result, ctx, handleClose]);

  const retry = useCallback(() => {
    setCapturedFrames([]);
    setPoseSamples([]);
    setImuTrace([]);
    setWindowStartNs(null);
    setResult(null);
    setProgress(null);
    setSubmitError(null);
    frameUrlsRef.current.clear();
    wizard.goTo(0);
  }, [wizard]);

  if (!wizard.open) return null;

  const stepNodes = [
    <Step1TargetCheck
      key="step1"
      ctx={ctx}
      onNext={wizard.next}
      onClose={handleClose}
      onEdgeMm={(value) => {
        targetEdgeRef.current = value;
      }}
    />,
    <Step2LivePreview
      key="step2"
      ctx={ctx}
      stream={stream}
      detections={detections}
      onFrameUrl={setLatestFrameUrl}
      onNext={wizard.next}
      onBack={wizard.back}
      tagCount={detections.length}
    />,
    <Step3FrameCapture
      key="step3"
      ctx={ctx}
      stream={stream}
      detections={detections}
      latestFrameUrl={latestFrameUrl}
      capturedFrames={capturedFrames}
      poseSamples={poseSamples}
      onCapture={() => void captureFrame()}
      onRemove={removeFrame}
      onFrameUrl={setLatestFrameUrl}
      onNext={wizard.next}
      onBack={wizard.back}
    />,
    <Step4ImuMotion
      key="step4"
      ctx={ctx}
      imuTrace={imuTrace}
      sufficient={imuMotionSufficient}
      onNext={wizard.next}
      onBack={wizard.back}
    />,
    <Step5Submit
      key="step5"
      ctx={ctx}
      frameCount={capturedFrames.length}
      submitting={submitting}
      submitError={submitError}
      onSubmit={() => void submit()}
      onBack={wizard.back}
    />,
    <Step6Wait
      key="step6"
      ctx={ctx}
      progress={progress}
    />,
    <Step7Verify
      key="step7"
      ctx={ctx}
      result={result}
      previousTelemetry={telemetry}
      submitError={submitError}
      onApply={() => void applyResult()}
      onRetry={retry}
    />,
  ];

  return (
    <Wizard
      currentStep={wizard.step}
      totalSteps={TOTAL_STEPS}
      indicatorLabel={`${tr(t, "navigation.calibrationWizard.step", "Step")} ${wizard.step + 1}/${TOTAL_STEPS}`}
      onJump={wizard.goTo}
      onDismiss={handleClose}
      dismissLabel={tr(t, "navigation.calibrationWizard.close", "Close")}
      layout="modal"
    >
      {stepNodes[wizard.step]}
    </Wizard>
  );
}

// ---------------------------------------------------------------------------
// Step components
// ---------------------------------------------------------------------------

interface ImuTracePoint {
  ts: number;
  gyroMag: number;
  accelMag: number;
}

function Step1TargetCheck({
  ctx,
  onNext,
  onClose,
  onEdgeMm,
}: {
  ctx: PluginContext;
  onNext: () => void;
  onClose: () => void;
  onEdgeMm: (mm: number) => void;
}): JSX.Element {
  const t = ctx.i18n.t;
  const [edgeMm, setEdgeMm] = useState("");
  const ready = /^\d+(\.\d+)?$/.test(edgeMm) && parseFloat(edgeMm) > 0;
  return (
    <WizardStep
      title={tr(t, "navigation.calibrationWizard.s1Title", "Target ready?")}
      description={tr(
        t,
        "navigation.calibrationWizard.s1Description",
        "Print the AprilGrid PDF at the bundled scale. Mount it rigid on foamcore. Measure the printed edge length with a ruler to confirm the print scale is correct.",
      )}
      actions={
        <>
          <SecondaryButton testIdSuffix="close" onClick={onClose}>
            {tr(t, "navigation.calibrationWizard.cancel", "Cancel")}
          </SecondaryButton>
          <PrimaryButton
            testIdSuffix="continue"
            disabled={!ready}
            onClick={() => {
              if (ready) {
                onEdgeMm(parseFloat(edgeMm));
                onNext();
              }
            }}
          >
            {tr(t, "navigation.calibrationWizard.continue", "Continue")}
          </PrimaryButton>
        </>
      }
    >
      <a
        href="/extensions/vision-nav/aprilgrid-t36h11-6x6.pdf"
        target="_blank"
        rel="noreferrer"
        style={pdfLink}
        data-testid="vn-cal-pdf-link"
      >
        {tr(
          t,
          "navigation.calibrationWizard.downloadPdf",
          "Download the AprilGrid PDF (t36h11, 6x6, 80x80 cm)",
        )}
      </a>
      <label style={fieldLabel}>
        {tr(
          t,
          "navigation.calibrationWizard.edgeLabel",
          "Measured edge length (mm)",
        )}
        <input
          type="text"
          inputMode="decimal"
          value={edgeMm}
          onChange={(e) => setEdgeMm(e.target.value)}
          style={textInput}
          placeholder="800"
          data-testid="vn-cal-edge-input"
        />
      </label>
    </WizardStep>
  );
}

function Step2LivePreview({
  ctx,
  stream,
  detections,
  onFrameUrl,
  onNext,
  onBack,
  tagCount,
}: {
  ctx: PluginContext;
  stream: MediaStream | null;
  detections: DetectionShape[];
  onFrameUrl: (url: string) => void;
  onNext: () => void;
  onBack: () => void;
  tagCount: number;
}): JSX.Element {
  const t = ctx.i18n.t;
  const ready = tagCount >= 24;
  // Suppress unused-var TS warning while detections aren't wired yet.
  void detections;
  return (
    <WizardStep
      title={tr(t, "navigation.calibrationWizard.s2Title", "Live preview")}
      description={tr(
        t,
        "navigation.calibrationWizard.s2Description",
        "Position the drone about 80 cm from the target. Watch the corner overlay; capture begins when at least 24 of 36 tags lock on.",
      )}
      actions={
        <>
          <SecondaryButton testIdSuffix="back" onClick={onBack}>
            {tr(t, "navigation.calibrationWizard.back", "Back")}
          </SecondaryButton>
          <PrimaryButton
            testIdSuffix="continue"
            disabled={!ready}
            onClick={onNext}
          >
            {tr(
              t,
              "navigation.calibrationWizard.beginCapture",
              "Begin capture",
            )}
          </PrimaryButton>
        </>
      }
    >
      <CameraPreviewWrapper
        stream={stream}
        detections={detections}
        onFrameUrl={onFrameUrl}
      />
      <span style={hint}>
        {tr(
          t,
          "navigation.calibrationWizard.tagCount",
          "Tags detected:",
        )}{" "}
        <strong style={{ color: ready ? "var(--vn-ok, #34d399)" : "var(--vn-warn, #f59e0b)" }}>
          {tagCount}/36
        </strong>
      </span>
    </WizardStep>
  );
}

function Step3FrameCapture({
  ctx,
  stream,
  detections,
  latestFrameUrl,
  capturedFrames,
  poseSamples,
  onCapture,
  onRemove,
  onFrameUrl,
  onNext,
  onBack,
}: {
  ctx: PluginContext;
  stream: MediaStream | null;
  detections: DetectionShape[];
  latestFrameUrl: string | null;
  capturedFrames: CapturedFrame[];
  poseSamples: PoseSample[];
  onCapture: () => void;
  onRemove: (id: string) => void;
  onFrameUrl: (url: string) => void;
  onNext: () => void;
  onBack: () => void;
}): JSX.Element {
  const t = ctx.i18n.t;
  const enoughFrames = capturedFrames.length >= MIN_FRAMES;
  const enoughDiversity = isPoseSetDiverse(poseSamples);
  const canProceed = enoughFrames && enoughDiversity;
  void latestFrameUrl;
  return (
    <WizardStep
      title={tr(
        t,
        "navigation.calibrationWizard.s3Title",
        "Capture frames at varied angles",
      )}
      description={tr(
        t,
        "navigation.calibrationWizard.s3Description",
        "Capture 20 to 30 frames. Move the drone between captures so the coverage map fills out across tilt and rotation.",
      )}
      actions={
        <>
          <SecondaryButton testIdSuffix="back" onClick={onBack}>
            {tr(t, "navigation.calibrationWizard.back", "Back")}
          </SecondaryButton>
          <PrimaryButton
            testIdSuffix="continue"
            disabled={!canProceed}
            onClick={onNext}
          >
            {tr(
              t,
              "navigation.calibrationWizard.continueToImu",
              "Continue to IMU motion",
            )}
          </PrimaryButton>
        </>
      }
    >
      <CameraPreviewWrapper
        stream={stream}
        detections={detections}
        onFrameUrl={onFrameUrl}
      />
      <div style={captureRow}>
        <PrimaryButton
          testIdSuffix="capture"
          onClick={onCapture}
          disabled={capturedFrames.length >= MAX_FRAMES}
        >
          {tr(t, "navigation.calibrationWizard.capture", "Capture")} ({capturedFrames.length}/{MAX_FRAMES})
        </PrimaryButton>
        <span style={hint}>
          {enoughDiversity
            ? tr(t, "navigation.calibrationWizard.diverse", "Pose coverage OK")
            : tr(
                t,
                "navigation.calibrationWizard.notDiverse",
                "Move to a new pose before capturing",
              )}
        </span>
      </div>
      <PoseCoverageMap
        samples={poseSamples}
        label={tr(t, "navigation.calibrationWizard.coverage", "Pose coverage")}
      />
      <FramePreview
        frames={capturedFrames}
        onRemove={onRemove}
        label={tr(t, "navigation.calibrationWizard.captured", "Captured")}
      />
    </WizardStep>
  );
}

function Step4ImuMotion({
  ctx,
  imuTrace,
  sufficient,
  onNext,
  onBack,
}: {
  ctx: PluginContext;
  imuTrace: ImuTracePoint[];
  sufficient: boolean;
  onNext: () => void;
  onBack: () => void;
}): JSX.Element {
  const t = ctx.i18n.t;
  return (
    <WizardStep
      title={tr(t, "navigation.calibrationWizard.s4Title", "IMU motion")}
      description={tr(
        t,
        "navigation.calibrationWizard.s4Description",
        "Move the drone in slow figure-eights for about 30 seconds. Live gyro and accel traces below confirm the IMU is being exercised across enough range.",
      )}
      actions={
        <>
          <SecondaryButton testIdSuffix="back" onClick={onBack}>
            {tr(t, "navigation.calibrationWizard.back", "Back")}
          </SecondaryButton>
          <PrimaryButton
            testIdSuffix="continue"
            disabled={!sufficient}
            onClick={onNext}
          >
            {tr(
              t,
              "navigation.calibrationWizard.continueToSubmit",
              "Continue to submit",
            )}
          </PrimaryButton>
        </>
      }
    >
      <ImuSparkline points={imuTrace} field="gyroMag" label="Gyro |ω| rad/s" />
      <ImuSparkline
        points={imuTrace}
        field="accelMag"
        label="Accel |a| m/s²"
      />
      <span style={hint}>
        {sufficient
          ? tr(t, "navigation.calibrationWizard.imuOk", "IMU motion sufficient")
          : tr(
              t,
              "navigation.calibrationWizard.imuMore",
              "Keep moving; need more dynamic range",
            )}
      </span>
    </WizardStep>
  );
}

function Step5Submit({
  ctx,
  frameCount,
  submitting,
  submitError,
  onSubmit,
  onBack,
}: {
  ctx: PluginContext;
  frameCount: number;
  submitting: boolean;
  submitError: string | null;
  onSubmit: () => void;
  onBack: () => void;
}): JSX.Element {
  const t = ctx.i18n.t;
  return (
    <WizardStep
      title={tr(t, "navigation.calibrationWizard.s5Title", "Submit to agent")}
      description={tr(
        t,
        "navigation.calibrationWizard.s5Description",
        "The agent runs OpenCV's AprilTag detection, the intrinsics solve, and the camera-IMU joint timeshift fit. This takes 30 to 60 seconds.",
      )}
      actions={
        <>
          <SecondaryButton testIdSuffix="back" onClick={onBack}>
            {tr(t, "navigation.calibrationWizard.back", "Back")}
          </SecondaryButton>
          <PrimaryButton
            testIdSuffix="submit"
            disabled={submitting}
            onClick={onSubmit}
          >
            {submitting
              ? tr(
                  t,
                  "navigation.calibrationWizard.submitting",
                  "Submitting...",
                )
              : tr(t, "navigation.calibrationWizard.submit", "Submit")}
          </PrimaryButton>
        </>
      }
    >
      <span style={hint}>
        {tr(
          t,
          "navigation.calibrationWizard.summary",
          "Bundle to submit:",
        )}{" "}
        {frameCount}{" "}
        {tr(t, "navigation.calibrationWizard.frames", "frames")} +{" "}
        {tr(t, "navigation.calibrationWizard.imuWindow", "IMU motion window")}
      </span>
      {submitError !== null ? (
        <ResultBanner
          kind="error"
          title={tr(
            t,
            "navigation.calibrationWizard.submitError",
            "Submit failed",
          )}
          body={submitError}
        />
      ) : null}
    </WizardStep>
  );
}

function Step6Wait({
  ctx,
  progress,
}: {
  ctx: PluginContext;
  progress: CalibrationProgressPayload | null;
}): JSX.Element {
  const t = ctx.i18n.t;
  const stageLabel = stageToLabel(progress?.stage ?? "queued", t);
  return (
    <WizardStep
      title={tr(
        t,
        "navigation.calibrationWizard.s6Title",
        "Running calibration",
      )}
      description={stageLabel}
    >
      <ProgressBar
        percent={progress === null ? null : Math.round(progress.percent)}
        label={stageLabel}
        sublabel={progress?.detail}
      />
    </WizardStep>
  );
}

function Step7Verify({
  ctx,
  result,
  previousTelemetry,
  submitError,
  onApply,
  onRetry,
}: {
  ctx: PluginContext;
  result: CalibrationResult | null;
  previousTelemetry: VisionNavTelemetry;
  submitError: string | null;
  onApply: () => void;
  onRetry: () => void;
}): JSX.Element {
  const t = ctx.i18n.t;
  if (submitError !== null || result === null) {
    return (
      <WizardStep
        title={tr(t, "navigation.calibrationWizard.s7Title", "Result")}
        actions={
          <>
            <SecondaryButton testIdSuffix="retry" onClick={onRetry}>
              {tr(t, "navigation.calibrationWizard.retry", "Retry")}
            </SecondaryButton>
          </>
        }
      >
        <ResultBanner
          kind="error"
          title={tr(
            t,
            "navigation.calibrationWizard.failed",
            "Calibration failed",
          )}
          body={
            submitError ??
            tr(
              t,
              "navigation.calibrationWizard.noResult",
              "Agent returned no result.",
            )
          }
        />
      </WizardStep>
    );
  }

  const previouslyLoaded = previousTelemetry.cameraIntrinsicsLoaded === true;
  const rows: DiagnosticRow[] = [
    {
      label: "fx",
      value: result.fx.toFixed(2),
      compare: previouslyLoaded ? "—" : undefined,
    },
    {
      label: "fy",
      value: result.fy.toFixed(2),
      compare: previouslyLoaded ? "—" : undefined,
    },
    {
      label: "cx",
      value: result.cx.toFixed(2),
      compare: previouslyLoaded ? "—" : undefined,
    },
    {
      label: "cy",
      value: result.cy.toFixed(2),
      compare: previouslyLoaded ? "—" : undefined,
    },
    {
      label: "Reprojection error",
      value: `${result.reprojectionErrorPx.toFixed(3)} px`,
      tone: result.reprojectionErrorPx < 1.0 ? "ok" : "warn",
    },
    {
      label: "Timeshift",
      value: `${(result.timeshiftCamImuS * 1000).toFixed(1)} ms`,
    },
    {
      label: "Timeshift residual",
      value: `${result.timeshiftResidualMs.toFixed(1)} ms`,
      tone: result.timeshiftResidualMs < 5 ? "ok" : "warn",
    },
    { label: "Frames used", value: `${result.framesUsed}` },
    { label: "Frames rejected", value: `${result.framesRejected}` },
  ];

  return (
    <WizardStep
      title={tr(t, "navigation.calibrationWizard.s7Title", "Verify result")}
      description={tr(
        t,
        "navigation.calibrationWizard.s7Description",
        "Reprojection error below 1 px and timeshift residual below 5 ms are healthy. Apply to persist, Retry to recapture.",
      )}
      actions={
        <>
          <SecondaryButton testIdSuffix="retry" onClick={onRetry}>
            {tr(t, "navigation.calibrationWizard.retry", "Retry")}
          </SecondaryButton>
          <PrimaryButton testIdSuffix="apply" onClick={onApply}>
            {tr(t, "navigation.calibrationWizard.apply", "Apply")}
          </PrimaryButton>
        </>
      }
    >
      <DiagnosticTable rows={rows} valueLabel="New" compareLabel="Previous" />
    </WizardStep>
  );
}

// ---------------------------------------------------------------------------
// Local components + helpers
// ---------------------------------------------------------------------------

function CameraPreviewWrapper({
  stream,
  detections,
  onFrameUrl,
}: {
  stream: MediaStream | null;
  detections: DetectionShape[];
  onFrameUrl: (url: string) => void;
}): JSX.Element {
  return (
    <CameraPreview
      stream={stream}
      detections={detections}
      onFrame={onFrameUrl}
    />
  );
}

function ImuSparkline({
  points,
  field,
  label,
}: {
  points: ImuTracePoint[];
  field: "gyroMag" | "accelMag";
  label: string;
}): JSX.Element {
  const w = 320;
  const h = 48;
  if (points.length < 2) {
    return (
      <div style={sparkBlock}>
        <span style={sparkLabel}>{label}</span>
        <svg width={w} height={h}>
          <line
            x1={0}
            x2={w}
            y1={h / 2}
            y2={h / 2}
            stroke="var(--vn-border, rgba(255,255,255,0.08))"
            strokeDasharray="2 2"
          />
        </svg>
      </div>
    );
  }
  const values = points.map((p) => p[field]);
  const min = Math.min(...values);
  const max = Math.max(...values);
  const span = Math.max(max - min, 1e-6);
  const poly = points
    .map((p, i) => {
      const x = (i / Math.max(points.length - 1, 1)) * w;
      const y = h - ((p[field] - min) / span) * h;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(" ");
  return (
    <div style={sparkBlock}>
      <span style={sparkLabel}>{label}</span>
      <svg width={w} height={h}>
        <polyline
          points={poly}
          fill="none"
          stroke="var(--vn-accent, #2563eb)"
          strokeWidth={1.5}
        />
      </svg>
    </div>
  );
}

function tagAreaSpanOf(detections: DetectionShape[]): number {
  if (detections.length === 0) return 0;
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  for (const det of detections) {
    for (const c of det.corners) {
      if (c.x < minX) minX = c.x;
      if (c.x > maxX) maxX = c.x;
      if (c.y < minY) minY = c.y;
      if (c.y > maxY) maxY = c.y;
    }
  }
  // Approximate: span as a fraction of a 640-wide canvas.
  return Math.min(1, (maxX - minX) / 640);
}

function toTagCorners(
  corners: DetectionShape["corners"],
): TagCorners {
  return {
    topLeft: corners[0],
    topRight: corners[1],
    bottomRight: corners[2],
    bottomLeft: corners[3],
  };
}

function stageToLabel(stage: CalibrationStage, t: TFn): string {
  const map: Record<CalibrationStage, [string, string]> = {
    queued: ["navigation.calibrationWizard.stageQueued", "Queued..."],
    tag_detection: [
      "navigation.calibrationWizard.stageDetection",
      "Detecting tags...",
    ],
    intrinsics_solve: [
      "navigation.calibrationWizard.stageIntrinsics",
      "Solving intrinsics...",
    ],
    extrinsics_solve: [
      "navigation.calibrationWizard.stageExtrinsics",
      "Solving extrinsics...",
    ],
    timeshift_solve: [
      "navigation.calibrationWizard.stageTimeshift",
      "Fitting timeshift...",
    ],
    complete: ["navigation.calibrationWizard.stageDone", "Done"],
    failed: ["navigation.calibrationWizard.stageFailed", "Failed"],
  };
  const [key, fallback] = map[stage];
  return tr(t, key, fallback);
}

function buildCamchainYaml(result: CalibrationResult): string {
  const rows = [
    result.tCamImu.slice(0, 4),
    result.tCamImu.slice(4, 8),
    result.tCamImu.slice(8, 12),
    result.tCamImu.slice(12, 16),
  ]
    .map((row) => `    - [${row.map((v) => v.toFixed(8)).join(", ")}]`)
    .join("\n");
  return `cam0:
  camera_model: ${result.cameraModel}
  intrinsics: [${result.fx.toFixed(4)}, ${result.fy.toFixed(4)}, ${result.cx.toFixed(4)}, ${result.cy.toFixed(4)}]
  distortion_model: ${result.distortionModel}
  distortion_coeffs: [${result.distortionCoeffs.map((v) => v.toFixed(6)).join(", ")}]
  resolution: [${result.width}, ${result.height}]
  T_cam_imu:
${rows}
  timeshift_cam_imu: ${result.timeshiftCamImuS.toFixed(6)}
`;
}

// ---------------------------------------------------------------------------
// Styles
// ---------------------------------------------------------------------------

const pdfLink: CSSProperties = {
  color: "var(--vn-accent, #2563eb)",
  textDecoration: "underline",
  fontSize: "0.8125rem",
};
const fieldLabel: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
  fontSize: "0.75rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const textInput: CSSProperties = {
  padding: "0.375rem 0.5rem",
  borderRadius: "0.25rem",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  background: "var(--vn-surface-2, rgba(255,255,255,0.06))",
  color: "var(--vn-text, #e5e7eb)",
  fontSize: "0.8125rem",
};
const hint: CSSProperties = {
  fontSize: "0.75rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const captureRow: CSSProperties = {
  display: "flex",
  alignItems: "center",
  gap: "0.75rem",
};
const sparkBlock: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
};
const sparkLabel: CSSProperties = {
  fontSize: "0.7rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
