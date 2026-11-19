/**
 * @module viewers/ViewerLoading
 * @description The loading overlay for a model viewer. With a known
 * download size it shows a determinate progress bar (percent + MB), so a
 * multi-hundred-MB reconstruction reads as real progress rather than an
 * indefinite spinner; with no size (or before the first byte) it falls back to
 * the spinner. Driven by the viewer that owns the download — cleared on the
 * first render or swapped for `ViewerError` on failure.
 */

import { Loader2 } from "lucide-react";

import { viewerT as t } from "../../../host/i18n";

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
      className="map:absolute map:inset-0 map:flex map:flex-col map:items-center map:justify-center map:gap-3 map:bg-bg-primary/40"
      role="status"
      aria-label={label ?? t("viewerStatus.loadingViewer")}
      aria-live="polite"
    >
      {determinate ? (
        <div className="map:flex map:w-48 map:max-w-[70%] map:flex-col map:gap-1.5">
          <div className="map:h-1 map:w-full map:overflow-hidden map:rounded-full map:bg-text-tertiary/20">
            <div
              className="map:h-full map:rounded-full map:bg-accent-primary map:transition-[width] map:duration-150"
              style={{ width: `${clamped}%` }}
            />
          </div>
          <div className="map:flex map:items-center map:justify-between map:text-[10px] map:font-mono map:tabular-nums map:text-text-tertiary">
            <span>{label ?? t("viewerStatus.loading")}</span>
            <span>
              {haveBytes
                ? `${toMb(receivedBytes)} / ${toMb(totalBytes)} MB`
                : `${Math.round(clamped)}%`}
            </span>
          </div>
        </div>
      ) : (
        <Loader2 className="map:w-5 map:h-5 map:animate-spin map:text-text-tertiary" />
      )}
    </div>
  );
}
