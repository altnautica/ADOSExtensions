import { useCallback, useEffect, useMemo, useState } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { TOUR_PERSIST_KEY, TOUR_STEPS } from "../tour/steps";
import type { VisionNavTelemetry } from "../types";

import { TourStep } from "./TourStep";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
  /** Forces the tour to open regardless of persistence. The NavigationTab
   * passes ``true`` when the operator clicks the "Replay tour" link. */
  replay: boolean;
  /** Notify the parent when the tour finishes or is skipped so the
   * parent can reset the Replay link state. */
  onClose: () => void;
}

/**
 * First-run guided tour for the Vision Navigation tab.
 *
 * Auto-opens on first mount when localStorage has no record of the
 * tour completing. Walks the operator through every visible card in
 * the order they will interact with the tab. Each step has an
 * optional pre-condition that skips the step when the telemetry does
 * not support it (e.g. the synthetic-fallback-banner preview step is
 * always shown; the VIO step is filtered out on agents without VIO
 * support).
 *
 * The component renders nothing when the tour is dismissed.
 */
export function TourOrchestrator({
  ctx,
  telemetry,
  replay,
  onClose,
}: Props): JSX.Element | null {
  // Filter the step list against the live telemetry so disabled steps
  // never appear. Memoised so the filtered list is stable across
  // re-renders when telemetry hasn't changed in a meaningful way.
  const steps = useMemo(
    () =>
      TOUR_STEPS.filter(
        (step) =>
          step.precondition === undefined || step.precondition(telemetry),
      ),
    [telemetry],
  );

  const [active, setActive] = useState<boolean>(false);
  const [stepIndex, setStepIndex] = useState<number>(0);

  // Decide whether to auto-open on mount. Reads the persistence flag
  // from localStorage; replay=true forces open regardless.
  useEffect(() => {
    if (replay) {
      setStepIndex(0);
      setActive(true);
      return;
    }
    try {
      const seen = window.localStorage.getItem(TOUR_PERSIST_KEY);
      if (seen !== "true") {
        setActive(true);
      }
    } catch {
      // Private-mode browsers reject localStorage; just default to
      // opening the tour. Slightly noisier UX but not broken.
      setActive(true);
    }
  }, [replay]);

  // Scroll the current step's target into view when the step changes.
  useEffect(() => {
    if (!active) return;
    const step = steps[stepIndex];
    if (step === undefined) return;
    try {
      const el = document.querySelector(
        `[data-testid="${step.targetTestId}"]`,
      );
      if (el instanceof HTMLElement) {
        el.scrollIntoView({
          behavior: "smooth",
          block: "center",
          inline: "nearest",
        });
      }
    } catch {
      // querySelector throws on invalid selectors; ours are static and
      // safe but the guard keeps us defensive.
    }
  }, [active, stepIndex, steps]);

  const close = useCallback(
    (markSeen: boolean) => {
      if (markSeen) {
        try {
          window.localStorage.setItem(TOUR_PERSIST_KEY, "true");
        } catch {
          // ignore
        }
      }
      setActive(false);
      setStepIndex(0);
      onClose();
    },
    [onClose],
  );

  if (!active) return null;
  if (steps.length === 0) {
    // No steps to show. Treat as if the tour completed; mark seen so
    // we do not auto-reopen later.
    close(true);
    return null;
  }

  const safeIndex = Math.min(stepIndex, steps.length - 1);
  const step = steps[safeIndex];
  if (step === undefined) {
    // Unreachable given the steps.length > 0 guard above, but the
    // strict TS index-access flag still wants the check.
    return null;
  }

  return (
    <TourStep
      step={step}
      index={safeIndex}
      total={steps.length}
      ctx={ctx}
      onNext={() => setStepIndex((n) => n + 1)}
      onSkip={() => close(true)}
      onFinish={() => close(true)}
    />
  );
}
