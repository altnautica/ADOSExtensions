/**
 * @module atlas/viewers/ViewerLoading
 * @description The loading overlay for a World Model viewer. With a known
 * download size it shows a determinate progress bar (percent + MB), so a
 * multi-hundred-MB reconstruction reads as real progress rather than an
 * indefinite spinner; with no size (or before the first byte) it falls back to
 * the spinner. Driven by the viewer that owns the download — cleared on the
 * first render or swapped for `ViewerError` on failure.
 */

import { Loader2 } from "lucide-react";

export interface ViewerLoadingProps {
  /** 0–100 when the download size is known; omit for an indeterminate spinner. */
  percent?: number;
  /** Bytes received so far (shown as MB alongside the total). */
  receivedBytes?: number;
  /** Total bytes when known (shown as MB). */
  totalBytes?: number;
  /** Short phase label, e.g. "Downloading splat". */
  label?: string;
}

const toMb = (bytes: number) => (bytes / (1024 * 1024)).toFixed(1);

export function ViewerLoading({
  percent,
  receivedBytes,
  totalBytes,
  label,
}: ViewerLoadingProps = {}) {
  const determinate = typeof percent === "number" && Number.isFinite(percent);
  const clamped = determinate ? Math.max(0, Math.min(100, percent)) : 0;
  const haveBytes =
    typeof receivedBytes === "number" &&
    typeof totalBytes === "number" &&
    totalBytes > 0;

  return (
    <div
      className="we:absolute we:inset-0 we:flex we:flex-col we:items-center we:justify-center we:gap-3 we:bg-bg-primary/40"
      role="status"
      aria-label={label ?? "Loading viewer"}
      aria-live="polite"
    >
      {determinate ? (
        <div className="we:flex we:w-48 we:max-w-[70%] we:flex-col we:gap-1.5">
          <div className="we:h-1 we:w-full we:overflow-hidden we:rounded-full we:bg-text-tertiary/20">
            <div
              className="we:h-full we:rounded-full we:bg-accent-primary we:transition-[width] we:duration-150"
              style={{ width: `${clamped}%` }}
            />
          </div>
          <div className="we:flex we:items-center we:justify-between we:text-[10px] we:font-mono we:tabular-nums we:text-text-tertiary">
            <span>{label ?? "Loading"}</span>
            <span>
              {haveBytes
                ? `${toMb(receivedBytes)} / ${toMb(totalBytes)} MB`
                : `${Math.round(clamped)}%`}
            </span>
          </div>
        </div>
      ) : (
        <Loader2 className="we:w-5 we:h-5 we:animate-spin we:text-text-tertiary" />
      )}
    </div>
  );
}
