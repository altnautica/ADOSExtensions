/**
 * Pose-diversity tracking for the calibration capture flow.
 *
 * The wizard captures 20-30 frames at varied poses. To produce a
 * well-conditioned intrinsics + extrinsics solve the captures need
 * to span multiple tilt angles and rotations. This module turns the
 * stream of captured poses into the data the PoseCoverageMap
 * primitive consumes.
 *
 * The pose is estimated cheaply from the tag corners the JS
 * AprilTag detector reports per frame. Tilt is the angle off-axis;
 * rotation is the in-plane rotation around the optical axis. Both
 * are derived from the homography of the target plane to the image
 * plane via four corresponding corners.
 */

import type { PoseSample } from "@altnautica/extension-ui";

/**
 * Four-corner shape matching ``DetectionShape.corners``. Re-declared
 * locally so the cluster module does not depend on the extension-ui
 * detection types directly.
 */
export interface TagCorners {
  topLeft: { x: number; y: number };
  topRight: { x: number; y: number };
  bottomRight: { x: number; y: number };
  bottomLeft: { x: number; y: number };
}

/**
 * Estimate tilt + rotation from a single tag's projected corners.
 *
 * Tilt is approximated as the ratio of the shorter side of the
 * projected quadrilateral to the longer side: a square front-on tag
 * gives ratio 1 (tilt 0); an obliquely-viewed tag gives ratio < 1
 * (tilt > 0). Rotation is the angle of the top edge relative to
 * horizontal.
 *
 * This is a rough heuristic; the agent's authoritative cv2 solver
 * does the precise pose recovery. We only need the rough buckets for
 * the coverage map.
 */
export function poseFromCorners(corners: TagCorners): PoseSample {
  const { topLeft, topRight, bottomRight, bottomLeft } = corners;

  // Side lengths (use the four edge lengths and pick the shorter/longer pair).
  const topLen = distance(topLeft, topRight);
  const bottomLen = distance(bottomLeft, bottomRight);
  const leftLen = distance(topLeft, bottomLeft);
  const rightLen = distance(topRight, bottomRight);

  const horizMean = (topLen + bottomLen) / 2;
  const vertMean = (leftLen + rightLen) / 2;
  const shorter = Math.min(horizMean, vertMean);
  const longer = Math.max(horizMean, vertMean);
  const ratio = longer === 0 ? 1 : shorter / longer;
  // Map ratio [1.0 .. 0.4] -> tilt [0 .. 90] linearly.
  const tiltDeg = Math.max(0, Math.min(90, (1 - ratio) * 150));

  // Rotation: angle of the top edge from horizontal.
  const dy = topRight.y - topLeft.y;
  const dx = topRight.x - topLeft.x;
  let rotationDeg = (Math.atan2(dy, dx) * 180) / Math.PI;
  if (rotationDeg < 0) rotationDeg += 360;

  return { tiltDeg, rotationDeg };
}

function distance(
  a: { x: number; y: number },
  b: { x: number; y: number },
): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
}

/**
 * Decide whether the captured pose set is diverse enough to proceed
 * to the next step. The simplest rule: at least 5 distinct buckets
 * (out of a 5x5 grid) need at least one sample.
 */
export function isPoseSetDiverse(
  samples: PoseSample[],
  gridSize = 5,
  minBuckets = 5,
): boolean {
  const buckets = new Set<number>();
  for (const sample of samples) {
    const rBucket = clampBucket(sample.rotationDeg, 0, 360, gridSize);
    const tBucket = clampBucket(sample.tiltDeg, 0, 90, gridSize);
    buckets.add(tBucket * gridSize + rBucket);
  }
  return buckets.size >= minBuckets;
}

function clampBucket(
  value: number,
  min: number,
  max: number,
  buckets: number,
): number {
  const span = max - min;
  if (span <= 0) return 0;
  const norm = (value - min) / span;
  const clamped = Math.max(0, Math.min(0.9999, norm));
  return Math.floor(clamped * buckets);
}
