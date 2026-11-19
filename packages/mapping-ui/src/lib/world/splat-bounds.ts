/**
 * @module world/splat-bounds
 * @description The box the viewer frames a splat by. A trained splat can
 * carry floaters: splats scattered far beyond the scene, up to a million
 * units out, transparent or collapsed to a flat disc of no thickness (and
 * often huge across). Their centres overflow Spark's float16 storage to
 * ±Infinity or reach the SPZ fixed-point limit, so the plain min/max of every
 * centre puts the camera so far out that the scene is sub-pixel and the view
 * looks empty. The framing box takes the splats a viewer sees as solid
 * (opacity at least `MIN_OPACITY`, smallest scale at least `THICKNESS_RATIO`
 * times the median middle scale of the opaque splats, so the test holds at
 * any model scale) and, per axis, the `TRIM` to `1 - TRIM` quantile of their
 * centres; with none it falls back to every finite centre.
 */

import type { Vec3 } from "./placement";

/** The opacity below which a splat does not count towards the framing box. */
export const MIN_OPACITY = 0.05;
/** A splat thinner than this fraction of the opaque splats' median middle
 * scale does not count: an axis trained to no thickness sits at its encoder's
 * floor (SPZ e^-10, Spark's packing e^-12), far under it. The middle scale,
 * not the largest, so huge flat floaters do not raise the floor. */
export const THICKNESS_RATIO = 1e-2;
/** The fraction of centres cut from each end of each axis. */
export const TRIM = 0.01;
/** At most this many centres are sorted; a larger splat is sampled evenly. */
const MAX_SAMPLES = 1 << 18;

type Xyz = { x: number; y: number; z: number };

/** Spark's `forEachSplat` signature, reduced to what the box needs. */
export type ForEachSplat = (
  callback: (index: number, center: Xyz, scales: Xyz, quaternion: unknown, opacity: number) => void,
) => void;

/** The framing box of a splat in its own frame, and whether any splat is
 * solid (else the box spans every finite centre and nothing would be seen). */
export interface SplatFraming {
  min: Vec3;
  max: Vec3;
  solid: boolean;
}

/** The framing of `count` splats walked (twice) by `forEach`, or null when
 * there is no finite centre. */
export function splatFramingBounds(forEach: ForEachSplat, count: number): SplatFraming | null {
  const step = Math.max(1, Math.ceil(count / MAX_SAMPLES));
  const cap = Math.ceil(count / step);
  const finiteCentre = (c: Xyz) => Number.isFinite(c.x) && Number.isFinite(c.y) && Number.isFinite(c.z);

  // Pass 1: the median middle scale of the opaque splats (the first `cap` of
  // them, so a sparse few still count) sets the thickness floor.
  const sizes = new Float64Array(cap);
  let nSizes = 0;
  forEach((_index, c, s, _q, opacity) => {
    if (!finiteCentre(c) || opacity < MIN_OPACITY || nSizes >= cap) return;
    sizes[nSizes] = s.x + s.y + s.z - Math.max(s.x, s.y, s.z) - Math.min(s.x, s.y, s.z);
    nSizes += 1;
  });
  const opaque = sizes.subarray(0, nSizes).sort();
  const minThickness = nSizes > 0 ? THICKNESS_RATIO * (opaque[nSizes >> 1] as number) : Infinity;

  // Pass 2: interleaved x, y, z of the sampled centres, the solid ones and all.
  const solid = new Float64Array(3 * cap);
  const every = new Float64Array(3 * cap);
  let nSolid = 0;
  let nEvery = 0;
  // Every splat is checked for solidity, sampled or not, so a few solid
  // splats among millions still count; the first stands in when none of them
  // was sampled.
  // (Typed by assertion: an assignment in the callback does not widen a `null` initializer.)
  let firstSolid = null as Vec3 | null;
  forEach((index, c, s, _q, opacity) => {
    if (!finiteCentre(c)) return;
    const isSolid = opacity >= MIN_OPACITY && Math.min(s.x, s.y, s.z) >= minThickness;
    if (isSolid && !firstSolid) firstSolid = [c.x, c.y, c.z];
    if (index % step !== 0 || nEvery >= cap) return;
    every[3 * nEvery] = c.x;
    every[3 * nEvery + 1] = c.y;
    every[3 * nEvery + 2] = c.z;
    nEvery += 1;
    if (isSolid) {
      solid[3 * nSolid] = c.x;
      solid[3 * nSolid + 1] = c.y;
      solid[3 * nSolid + 2] = c.z;
      nSolid += 1;
    }
  });
  const anySolid = firstSolid !== null;
  if (firstSolid && nSolid === 0) {
    solid.set(firstSolid);
    nSolid = 1;
  }
  const [xyz, n] = nSolid > 0 ? [solid, nSolid] : [every, nEvery];
  if (n === 0) return null;
  const lo = Math.floor(TRIM * (n - 1));
  const hi = Math.ceil((1 - TRIM) * (n - 1));
  const axis = new Float64Array(n);
  const bound = (a: number): [number, number] => {
    for (let i = 0; i < n; i++) axis[i] = xyz[3 * i + a] as number;
    axis.sort();
    return [axis[lo] as number, axis[hi] as number];
  };
  const [x, y, z] = [bound(0), bound(1), bound(2)];
  return { min: [x[0], y[0], z[0]], max: [x[1], y[1], z[1]], solid: anySolid };
}
