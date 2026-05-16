/**
 * Wire types for the calibration round-trip.
 *
 * GCS publishes ``start_calibration`` with the frame bundle + the
 * IMU recording window. Agent publishes ``calibration_progress``
 * substep updates and ``calibration_complete`` with the final
 * intrinsics + extrinsics + diagnostics.
 *
 * All payloads serialise as plain JSON; the agent's runner reads
 * them as Python dicts. Camera frames travel base64-encoded inside
 * the start_calibration payload to avoid a separate binary upload
 * channel.
 */

export interface StartCalibrationPayload {
  type: "start_calibration";
  /** Base64-encoded PNG frames the operator captured. */
  framesB64: string[];
  /** Capture window start time in agent-monotonic-nanoseconds. */
  windowStartNs: number;
  /** Capture window end time in agent-monotonic-nanoseconds. */
  windowEndNs: number;
  /** Camera resolution the captures were made at. */
  width: number;
  height: number;
}

export type CalibrationStage =
  | "queued"
  | "tag_detection"
  | "intrinsics_solve"
  | "extrinsics_solve"
  | "timeshift_solve"
  | "complete"
  | "failed";

export interface CalibrationProgressPayload {
  type: "calibration_progress";
  stage: CalibrationStage;
  percent: number;
  /** Optional substep diagnostic the wizard surfaces in real-time. */
  detail?: string;
}

export interface CalibrationResult {
  cameraModel: "pinhole";
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  width: number;
  height: number;
  distortionModel: "radtan" | "none";
  distortionCoeffs: number[];
  tCamImu: number[]; // 16 floats, row-major
  timeshiftCamImuS: number;
  reprojectionErrorPx: number;
  timeshiftResidualMs: number;
  framesUsed: number;
  framesRejected: number;
}

export interface CalibrationCompletePayload {
  type: "calibration_complete";
  result: CalibrationResult | null;
  /** Filled when the agent failed to converge. */
  error?: string;
}
