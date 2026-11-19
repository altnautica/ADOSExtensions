import { useEffect, useRef, type CSSProperties, type KeyboardEvent } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { tr } from "../i18n";
import type { TourStep as StepSpec } from "../tour/steps";

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
 * One step's overlay + content card, as a modal dialog: focus moves to
 * the primary button on each step and Escape skips the tour. The
 * orchestrator owns the step index and the persistence.
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
  const primary = useRef<HTMLButtonElement>(null);
  const titleId = `vn-tour-title-${step.id}`;

  useEffect(() => {
    primary.current?.focus();
  }, [step.id]);

  const onKeyDown = (e: KeyboardEvent<HTMLDivElement>): void => {
    if (e.key === "Escape") {
      e.stopPropagation();
      onSkip();
    }
  };

  return (
    <div style={overlay} data-testid="vn-tour-overlay" onKeyDown={onKeyDown}>
      <div
        style={card}
        role="dialog"
        aria-modal="true"
        aria-labelledby={titleId}
        data-testid={`vn-tour-step-${step.id}`}
      >
        <header style={header}>
          <span style={counter}>{tr(t, "navigation.tour.step", { n: index + 1, total })}</span>
          <button type="button" style={skipBtn} onClick={onSkip} data-testid="vn-tour-skip">
            {tr(t, "navigation.tour.skip")}
          </button>
        </header>
        <h3 style={title} id={titleId}>
          {tr(t, step.titleKey)}
        </h3>
        <p style={body}>{tr(t, step.bodyKey)}</p>
        <div style={actions}>
          <button
            ref={primary}
            type="button"
            style={primaryBtn}
            onClick={isLast ? onFinish : onNext}
            data-testid={isLast ? "vn-tour-finish" : "vn-tour-next"}
          >
            {tr(t, isLast ? "navigation.tour.finish" : "navigation.tour.next")}
          </button>
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
  background: "var(--vn-surface, #0a0a0a)",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.12))",
  borderRadius: "0.5rem",
  padding: "1rem 1.125rem",
  color: "var(--vn-text, #ededed)",
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
  color: "var(--vn-text-2, #b4b4b4)",
  lineHeight: 1.45,
};
const actions: CSSProperties = {
  display: "flex",
  justifyContent: "flex-end",
};
// Both buttons keep a 48 px hit area for touch.
const primaryBtn: CSSProperties = {
  padding: "0.5rem 1rem",
  minHeight: "48px",
  minWidth: "48px",
  background: "var(--vn-accent, #3a82ff)",
  color: "white",
  border: "none",
  borderRadius: "0.375rem",
  fontWeight: 600,
  cursor: "pointer",
  fontSize: "0.8125rem",
};
const skipBtn: CSSProperties = {
  padding: "0.25rem 0.75rem",
  minHeight: "48px",
  minWidth: "48px",
  background: "transparent",
  color: "var(--vn-text-muted, #94a3b8)",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.25rem",
  cursor: "pointer",
  fontSize: "0.8125rem",
};
