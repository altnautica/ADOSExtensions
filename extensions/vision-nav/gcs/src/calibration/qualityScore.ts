/**
 * Per-frame quality scoring for calibration captures.
 *
 * The flagship wizard refuses to keep frames that fail this gate. The
 * operator sees a GOOD / OK / DROP chip per captured frame and a
 * Keep button that enables only on GOOD or OK.
 *
 * Three signals roll into the score:
 *
 * 1. **Sharpness** via the Laplacian-variance proxy. Blurry frames
 *    have low pixel-level variance after a high-pass filter pass; we
 *    compute the variance of a 3x3 Laplacian convolution on a
 *    downsampled grayscale frame.
 *
 * 2. **Tag count.** The detector reports how many of the 36
 *    AprilGrid tags were locked onto. Fewer than 24 visible tags
 *    means the calibration math will have too few constraints.
 *
 * 3. **Tag-area span.** The captured frame should show the target
 *    filling at least 60% of one frame dimension; otherwise the tag
 *    corners cluster and the intrinsics solve becomes ill-conditioned.
 *
 * The three signals are folded into a categorical GOOD / OK / DROP
 * verdict; the wizard ignores DROP frames.
 */

export type FrameQualityVerdict = "good" | "ok" | "drop";

export interface FrameQualitySignals {
  /** Variance of the 3x3 Laplacian convolution. */
  sharpness: number;
  /** Number of AprilTags detected (out of 36 for the t36h11 6x6 grid). */
  tagCount: number;
  /** Largest tag-corner span as a fraction of frame width [0..1]. */
  tagAreaSpan: number;
  /** Mean exposure of the captured frame [0..255]. */
  meanExposure: number;
}

export interface FrameQualityResult {
  verdict: FrameQualityVerdict;
  signals: FrameQualitySignals;
  reasons: string[];
}

const SHARPNESS_GOOD = 800;
const SHARPNESS_DROP = 200;
const TAG_COUNT_GOOD = 30;
const TAG_COUNT_DROP = 24;
const SPAN_GOOD = 0.6;
const SPAN_DROP = 0.4;
const EXPOSURE_MIN = 30;
const EXPOSURE_MAX = 225;

/**
 * Compute a quality verdict from the four signals. Pure function so
 * the call site can drive it with either real signals (canvas-based
 * sharpness + JS detector tag corners) or synthetic ones in demo
 * mode.
 */
export function scoreFrame(
  signals: FrameQualitySignals,
): FrameQualityResult {
  const reasons: string[] = [];

  if (signals.sharpness < SHARPNESS_DROP) {
    reasons.push("blurry");
  }
  if (signals.tagCount < TAG_COUNT_DROP) {
    reasons.push("too few tags");
  }
  if (signals.tagAreaSpan < SPAN_DROP) {
    reasons.push("target too small in frame");
  }
  if (
    signals.meanExposure < EXPOSURE_MIN ||
    signals.meanExposure > EXPOSURE_MAX
  ) {
    reasons.push("exposure out of range");
  }

  if (reasons.length > 0) {
    return { verdict: "drop", signals, reasons };
  }

  const isGood =
    signals.sharpness >= SHARPNESS_GOOD &&
    signals.tagCount >= TAG_COUNT_GOOD &&
    signals.tagAreaSpan >= SPAN_GOOD;
  return {
    verdict: isGood ? "good" : "ok",
    signals,
    reasons,
  };
}

/**
 * Compute Laplacian-variance sharpness from a canvas data URL. Runs
 * in the browser; downsamples to 320x240 grayscale for speed (the
 * absolute value of the variance does not need full-resolution
 * precision for the GOOD/OK/DROP gate).
 *
 * Caller responsibility: pass a data URL produced from the same
 * canvas the user saw at capture time. The function decodes the
 * image asynchronously.
 */
export async function sharpnessFromDataUrl(
  dataUrl: string,
): Promise<number> {
  return new Promise<number>((resolve) => {
    const img = new Image();
    img.onload = () => {
      const w = 320;
      const h = 240;
      const canvas = document.createElement("canvas");
      canvas.width = w;
      canvas.height = h;
      const ctx = canvas.getContext("2d");
      if (ctx === null) {
        resolve(0);
        return;
      }
      ctx.drawImage(img, 0, 0, w, h);
      const data = ctx.getImageData(0, 0, w, h).data;
      const gray = new Float32Array(w * h);
      for (let i = 0; i < w * h; i++) {
        const r = data[i * 4] ?? 0;
        const g = data[i * 4 + 1] ?? 0;
        const b = data[i * 4 + 2] ?? 0;
        gray[i] = 0.2126 * r + 0.7152 * g + 0.0722 * b;
      }
      // 3x3 Laplacian convolution; collect variance of the output.
      let sum = 0;
      let sumSq = 0;
      let count = 0;
      for (let y = 1; y < h - 1; y++) {
        for (let x = 1; x < w - 1; x++) {
          const idx = y * w + x;
          const lap =
            -1 * (gray[idx - w - 1] ?? 0) +
            -1 * (gray[idx - w] ?? 0) +
            -1 * (gray[idx - w + 1] ?? 0) +
            -1 * (gray[idx - 1] ?? 0) +
            8 * (gray[idx] ?? 0) +
            -1 * (gray[idx + 1] ?? 0) +
            -1 * (gray[idx + w - 1] ?? 0) +
            -1 * (gray[idx + w] ?? 0) +
            -1 * (gray[idx + w + 1] ?? 0);
          sum += lap;
          sumSq += lap * lap;
          count++;
        }
      }
      const mean = sum / Math.max(count, 1);
      const variance = sumSq / Math.max(count, 1) - mean * mean;
      resolve(Math.max(0, variance));
    };
    img.onerror = () => resolve(0);
    img.src = dataUrl;
  });
}

/**
 * Compute mean exposure (mean of grayscale luminance) from a data URL.
 * Used by :func:`scoreFrame` as one of the four quality signals.
 */
export async function meanExposureFromDataUrl(
  dataUrl: string,
): Promise<number> {
  return new Promise<number>((resolve) => {
    const img = new Image();
    img.onload = () => {
      const canvas = document.createElement("canvas");
      canvas.width = 64;
      canvas.height = 64;
      const ctx = canvas.getContext("2d");
      if (ctx === null) {
        resolve(0);
        return;
      }
      ctx.drawImage(img, 0, 0, 64, 64);
      const data = ctx.getImageData(0, 0, 64, 64).data;
      let total = 0;
      const n = 64 * 64;
      for (let i = 0; i < n; i++) {
        const r = data[i * 4] ?? 0;
        const g = data[i * 4 + 1] ?? 0;
        const b = data[i * 4 + 2] ?? 0;
        total += 0.2126 * r + 0.7152 * g + 0.0722 * b;
      }
      resolve(total / n);
    };
    img.onerror = () => resolve(0);
    img.src = dataUrl;
  });
}
