import type { CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

export interface CapturedFrame {
  id: string;
  /** PNG data URL (or any browser-renderable image src). */
  src: string;
  /** Optional quality tag rendered as a corner chip. */
  quality?: "good" | "ok" | "drop";
}

interface Props {
  frames: CapturedFrame[];
  /** Optional remove callback. When provided each thumbnail shows a
   * delete button so the operator can drop a bad capture. */
  onRemove?: (id: string) => void;
  /** Optional label rendered above the grid. */
  label?: string;
}

/**
 * Thumbnail grid of captured frames. Used by the calibration wizard
 * to show the operator what they have captured so far + let them
 * delete bad frames before submitting.
 */
export function FramePreview({
  frames,
  onRemove,
  label,
}: Props): JSX.Element {
  return (
    <div style={wrapper} data-testid="ext-ui-frame-preview">
      {label !== undefined ? <span style={labelStyle}>{label}</span> : null}
      <div style={grid}>
        {frames.map((f) => (
          <div
            key={f.id}
            style={cell}
            data-testid={`ext-ui-frame-preview-thumb-${f.id}`}
          >
            <img src={f.src} alt={`Frame ${f.id}`} style={img} />
            {f.quality !== undefined ? (
              <span style={badge(f.quality)}>
                {f.quality.toUpperCase()}
              </span>
            ) : null}
            {onRemove !== undefined ? (
              <button
                type="button"
                style={removeBtn}
                onClick={() => onRemove(f.id)}
                data-testid={`ext-ui-frame-preview-remove-${f.id}`}
                aria-label={`Remove frame ${f.id}`}
              >
                ✕
              </button>
            ) : null}
          </div>
        ))}
        {frames.length === 0 ? (
          <span style={emptyState}>No frames captured yet.</span>
        ) : null}
      </div>
    </div>
  );
}

const QUALITY_COLOR = {
  good: TOKENS.ok,
  ok: TOKENS.warn,
  drop: TOKENS.error,
} as const;

const wrapper: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.375rem",
};
const labelStyle: CSSProperties = {
  fontSize: "0.75rem",
  color: TOKENS.textMuted,
  fontWeight: 600,
};
const grid: CSSProperties = {
  display: "grid",
  gridTemplateColumns: "repeat(auto-fit, minmax(90px, 1fr))",
  gap: "0.5rem",
};
const cell: CSSProperties = {
  position: "relative",
  aspectRatio: "4 / 3",
  background: TOKENS.surface2,
  borderRadius: "0.25rem",
  overflow: "hidden",
};
const img: CSSProperties = {
  width: "100%",
  height: "100%",
  objectFit: "cover",
  display: "block",
};
const badge = (kind: "good" | "ok" | "drop"): CSSProperties => ({
  position: "absolute",
  top: "0.25rem",
  left: "0.25rem",
  padding: "0.1rem 0.3rem",
  borderRadius: "0.2rem",
  background: QUALITY_COLOR[kind],
  color: "white",
  fontSize: "0.6rem",
  fontWeight: 700,
  letterSpacing: "0.04em",
});
const removeBtn: CSSProperties = {
  position: "absolute",
  top: "0.25rem",
  right: "0.25rem",
  width: "1.25rem",
  height: "1.25rem",
  borderRadius: "999px",
  border: "none",
  background: "rgba(0, 0, 0, 0.65)",
  color: "white",
  cursor: "pointer",
  fontSize: "0.7rem",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
};
const emptyState: CSSProperties = {
  gridColumn: "1 / -1",
  fontSize: "0.75rem",
  color: TOKENS.textMuted,
  textAlign: "center",
  padding: "1rem 0",
};
