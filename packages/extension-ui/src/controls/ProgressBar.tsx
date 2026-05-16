import { useEffect, useRef, useState, type CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

interface Props {
  /** 0 to 100 inclusive. ``null`` switches the bar to indeterminate. */
  percent: number | null;
  /** Optional label shown above the bar. */
  label?: string;
  /** Optional sublabel shown below the bar (e.g. "42 / 100 frames"). */
  sublabel?: string;
}

/**
 * Determinate + indeterminate progress bar. Determinate renders a
 * filled bar to the right percentage; indeterminate animates a
 * sliding strip with a 1.5s cycle.
 */
export function ProgressBar({ percent, label, sublabel }: Props): JSX.Element {
  const indeterminate = percent === null;
  const clamped = indeterminate ? 0 : Math.max(0, Math.min(100, percent));
  const sliderRef = useRef<HTMLDivElement | null>(null);
  const [slide, setSlide] = useState(0);

  // Animate the indeterminate slider via requestAnimationFrame so the
  // motion stays smooth without a CSS keyframe injection (which the
  // iframe sandbox may strip from <style> tags in some hosts).
  useEffect(() => {
    if (!indeterminate) return;
    let raf = 0;
    const start = performance.now();
    function tick(now: number) {
      const t = ((now - start) / 1500) % 1;
      // Slider moves from -30% to +130% so the bar fills then exits.
      setSlide(-30 + t * 160);
      raf = window.requestAnimationFrame(tick);
    }
    raf = window.requestAnimationFrame(tick);
    return () => window.cancelAnimationFrame(raf);
  }, [indeterminate]);

  return (
    <div style={wrapper} data-testid="ext-ui-progress-bar">
      {label !== undefined ? <span style={labelStyle}>{label}</span> : null}
      <div style={track}>
        {indeterminate ? (
          <div
            ref={sliderRef}
            style={{
              ...fillBase,
              width: "30%",
              transform: `translateX(${slide}%)`,
            }}
          />
        ) : (
          <div
            style={{
              ...fillBase,
              width: `${clamped}%`,
              transition: "width 240ms ease",
            }}
            role="progressbar"
            aria-valuemin={0}
            aria-valuemax={100}
            aria-valuenow={Math.round(clamped)}
          />
        )}
      </div>
      {sublabel !== undefined ? (
        <span style={sublabelStyle}>{sublabel}</span>
      ) : null}
    </div>
  );
}

const wrapper: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
  width: "100%",
};
const labelStyle: CSSProperties = {
  fontSize: "0.75rem",
  color: TOKENS.textMuted,
  fontWeight: 600,
};
const sublabelStyle: CSSProperties = {
  fontSize: "0.7rem",
  color: TOKENS.textMuted,
};
const track: CSSProperties = {
  position: "relative",
  height: "0.5rem",
  background: TOKENS.surface2,
  borderRadius: "999px",
  overflow: "hidden",
};
const fillBase: CSSProperties = {
  height: "100%",
  background: TOKENS.accent,
  borderRadius: "999px",
};
