/**
 * @module world/streamed-framing
 * @description When to frame a streamed LoD splat. Its RAD header carries no
 * extent, and the renderer pages splats in by view, starting near the default
 * camera, so framing once on the first pages leaves the orbit camera inside a
 * small corner of the scene. Instead the stage re-frames while the paged count
 * grows (at most every `REFRAME_MS`), until it holds for `STABLE_MS` or
 * `SETTLE_MS` have passed since the first pages, and never after the user
 * has moved the camera.
 */

/** The least time between two re-framings. */
export const REFRAME_MS = 250;
/** A paged count unchanged this long has settled. */
export const STABLE_MS = 600;
/** Re-framing stops this long after the first pages, settled or not. */
export const SETTLE_MS = 2000;

export type FramingStep = "frame" | "wait" | "done";

export class StreamedFraming {
  private firstAt: number | null = null;
  private grewAt = 0;
  private framedAt = -Infinity;
  private framedCount = 0;
  private seenCount = 0;
  private stopped = false;

  /** The user moved the camera: their view wins from here on. */
  stop(): void {
    this.stopped = true;
  }

  /** One rendered frame with `count` splats paged in at `now` (ms): whether to
   * re-frame on them now, wait, or stop for good. */
  step(count: number, now: number): FramingStep {
    if (this.stopped) return "done";
    if (count === 0) return "wait";
    if (this.firstAt === null) this.firstAt = now;
    if (count !== this.seenCount) {
      this.seenCount = count;
      this.grewAt = now;
    }
    const settled = now - this.grewAt >= STABLE_MS || now - this.firstAt >= SETTLE_MS;
    if (count !== this.framedCount && (settled || now - this.framedAt >= REFRAME_MS)) {
      this.framedAt = now;
      this.framedCount = count;
      if (settled) this.stopped = true;
      return "frame";
    }
    if (settled) {
      this.stopped = true;
      return "done";
    }
    return "wait";
  }
}
