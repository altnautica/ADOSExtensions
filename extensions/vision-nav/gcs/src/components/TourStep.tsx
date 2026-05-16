import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { TourStep as StepSpec } from "../tour/steps";

type TFn = PluginContext["i18n"]["t"];

function tr(t: TFn, key: string, fallback: string): string {
  const result = t(key);
  return result === key ? fallback : result;
}

interface Props {
  step: StepSpec;
  index: number;
  total: number;
  ctx: PluginContext;
  onNext: () => void;
  onSkip: () => void;
  onFinish: () => void;
}

/**
 * One step's overlay + content card. The orchestrator owns the
 * step index and the persistence; this component renders the card
 * and exposes Next / Skip / Finish.
 */
export function TourStep({
  step,
  index,
  total,
  ctx,
  onNext,
  onSkip,
  onFinish,
}: Props): JSX.Element {
  const t = ctx.i18n.t;
  const isLast = index === total - 1;
  return (
    <div style={overlay} data-testid="vn-tour-overlay">
      <div style={card} data-testid={`vn-tour-step-${step.id}`}>
        <header style={header}>
          <span style={counter}>
            {tr(t, "navigation.tour.step", "Step")} {index + 1}/{total}
          </span>
          <button
            type="button"
            style={skipBtn}
            onClick={onSkip}
            data-testid="vn-tour-skip"
          >
            {tr(t, "navigation.tour.skip", "Skip")}
          </button>
        </header>
        <h3 style={title}>{tr(t, step.titleKey, step.titleFallback)}</h3>
        <p style={body}>{tr(t, step.bodyKey, step.bodyFallback)}</p>
        <div style={actions}>
          {isLast ? (
            <button
              type="button"
              style={primaryBtn}
              onClick={onFinish}
              data-testid="vn-tour-finish"
            >
              {tr(t, "navigation.tour.finish", "Finish")}
            </button>
          ) : (
            <button
              type="button"
              style={primaryBtn}
              onClick={onNext}
              data-testid="vn-tour-next"
            >
              {tr(t, "navigation.tour.next", "Next")}
            </button>
          )}
        </div>
      </div>
    </div>
  );
}

const overlay: CSSProperties = {
  position: "fixed",
  top: 0,
  left: 0,
  right: 0,
  bottom: 0,
  background: "rgba(0, 0, 0, 0.5)",
  display: "flex",
  alignItems: "flex-end",
  justifyContent: "center",
  padding: "1.5rem",
  zIndex: 50,
};
const card: CSSProperties = {
  maxWidth: "420px",
  width: "100%",
  background: "var(--vn-surface, #0b1220)",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.12))",
  borderRadius: "0.5rem",
  padding: "1rem 1.125rem",
  color: "var(--vn-text, #e5e7eb)",
  boxShadow: "0 10px 32px rgba(0, 0, 0, 0.4)",
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
};
const header: CSSProperties = {
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
};
const counter: CSSProperties = {
  fontSize: "0.7rem",
  color: "var(--vn-text-muted, #94a3b8)",
  fontWeight: 600,
};
const title: CSSProperties = {
  fontSize: "0.9375rem",
  fontWeight: 600,
  margin: 0,
};
const body: CSSProperties = {
  fontSize: "0.8125rem",
  margin: 0,
  color: "var(--vn-text-2, #cbd5e1)",
  lineHeight: 1.45,
};
const actions: CSSProperties = {
  display: "flex",
  justifyContent: "flex-end",
};
const primaryBtn: CSSProperties = {
  padding: "0.5rem 1rem",
  background: "var(--vn-accent, #2563eb)",
  color: "white",
  border: "none",
  borderRadius: "0.375rem",
  fontWeight: 600,
  cursor: "pointer",
  fontSize: "0.8125rem",
};
const skipBtn: CSSProperties = {
  padding: "0.25rem 0.5rem",
  background: "transparent",
  color: "var(--vn-text-muted, #94a3b8)",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.25rem",
  cursor: "pointer",
  fontSize: "0.7rem",
};
