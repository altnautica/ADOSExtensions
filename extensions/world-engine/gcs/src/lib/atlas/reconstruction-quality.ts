/**
 * @module lib/atlas/reconstruction-quality
 * @description Human-intuitive "detail level" presets for a gaussian-splat
 * reconstruction. Each level bundles the real Brush knobs that trade quality for
 * speed: training steps, a gaussian-count cap (`max_splats`), and SH degree.
 * Bounding the gaussian count is what keeps training fast — left uncapped, the
 * trainer densifies a scene to millions of splats and each step slows to a crawl
 * (hours); the cap holds it to a budget (minutes) with little visible loss.
 *
 * The operator picks a level (Draft / Standard / High / Maximum) on the drone
 * tab where a reconstruction is commissioned; the choice rides the reconstruct
 * job's `params` (steps + max_splats + sh_degree), honored per-job by the compute
 * node. `qualityForSteps` decodes an existing job's step count back to the
 * nearest level so a finished artifact can be labelled with its detail level.
 */

export type ReconstructionQualityId = "draft" | "standard" | "high" | "maximum";

export interface ReconstructionQuality {
  id: ReconstructionQualityId;
  /** Brush training iterations for this level. */
  steps: number;
  /** Gaussian-count cap (`--max-splats`) — the primary speed/quality lever. */
  maxSplats: number;
  /** Spherical-harmonics degree (`--sh-degree`, 0-3). Lower = a bit faster, less
   * view-dependent colour. */
  shDegree: number;
  /** Display order, coarsest → finest. */
  order: number;
  /** i18n key (under `atlas.reconstructQuality`) for the short label. */
  labelKey: string;
  /** i18n key for the one-line description. */
  descKey: string;
}

/** The detail levels, coarsest → finest. */
export const RECONSTRUCTION_QUALITIES: readonly ReconstructionQuality[] = [
  {
    id: "draft",
    steps: 7000,
    maxSplats: 600_000,
    shDegree: 2,
    order: 0,
    labelKey: "reconstructQuality.draftLabel",
    descKey: "reconstructQuality.draftDesc",
  },
  {
    id: "standard",
    steps: 15000,
    maxSplats: 1_000_000,
    shDegree: 3,
    order: 1,
    labelKey: "reconstructQuality.standardLabel",
    descKey: "reconstructQuality.standardDesc",
  },
  {
    id: "high",
    steps: 30000,
    maxSplats: 1_500_000,
    shDegree: 3,
    order: 2,
    labelKey: "reconstructQuality.highLabel",
    descKey: "reconstructQuality.highDesc",
  },
  {
    id: "maximum",
    steps: 50000,
    maxSplats: 2_500_000,
    shDegree: 3,
    order: 3,
    labelKey: "reconstructQuality.maximumLabel",
    descKey: "reconstructQuality.maximumDesc",
  },
];

/** The recommended default (the Brush / 3DGS full-quality baseline). */
export const DEFAULT_QUALITY_ID: ReconstructionQualityId = "high";

/** The default step count (the `high` level). Used when a drone has no persisted
 * `reconstruct_steps` yet. */
export const DEFAULT_RECONSTRUCTION_STEPS = 30000;

/** The preset for an id (defaults to `high` for an unknown id). */
export function qualityById(id: string): ReconstructionQuality {
  return (
    RECONSTRUCTION_QUALITIES.find((q) => q.id === id) ??
    RECONSTRUCTION_QUALITIES.find((q) => q.id === DEFAULT_QUALITY_ID)!
  );
}

/** The step count for a preset id. */
export function stepsForQuality(id: string): number {
  return qualityById(id).steps;
}

/**
 * The nearest preset for an arbitrary step count — used to LABEL an existing
 * job whose `params.steps` may not exactly equal a preset (an older 7k job, a
 * hand-submitted custom count). Nearest by absolute distance; ties favour the
 * finer level.
 */
export function qualityForSteps(steps: number): ReconstructionQuality {
  let best = qualityById(DEFAULT_QUALITY_ID);
  let bestDist = Number.POSITIVE_INFINITY;
  for (const q of RECONSTRUCTION_QUALITIES) {
    const d = Math.abs(steps - q.steps);
    if (d < bestDist || (d === bestDist && q.steps > best.steps)) {
      best = q;
      bestDist = d;
    }
  }
  return best;
}
