import { useCallback, useEffect, useState } from "react";

/**
 * State + transitions for a multi-step wizard.
 *
 * The hook owns the current step index, whether the wizard is open,
 * and exposes advance / back / dismiss / open methods. Optional
 * persistence to localStorage gives the "tour seen" behaviour the
 * tour orchestrator relies on; calibration wizards skip persistence
 * because every calibration run starts fresh.
 */
export interface WizardStateOptions {
  /** Total step count. ``advance`` clamps to this maximum. */
  totalSteps: number;
  /** Persist completion to localStorage under this key. */
  persistenceKey?: string;
  /** Open the wizard automatically on first mount when persistence is
   * empty. Default: false. */
  autoOpenOnFirstRun?: boolean;
  /** Force the wizard open regardless of persistence. Used by the
   * consumer's "Replay" link. */
  replay?: boolean;
}

export interface WizardState {
  open: boolean;
  step: number;
  isLast: boolean;
  next: () => void;
  back: () => void;
  goTo: (step: number) => void;
  dismiss: (markSeen?: boolean) => void;
  open_: () => void;
}

export function useWizardState(opts: WizardStateOptions): WizardState {
  const {
    totalSteps,
    persistenceKey,
    autoOpenOnFirstRun = false,
    replay = false,
  } = opts;

  const [open, setOpen] = useState(false);
  const [step, setStep] = useState(0);

  // Decide on first mount whether to auto-open. Reads persistence
  // when configured; replay always wins.
  useEffect(() => {
    if (replay) {
      setStep(0);
      setOpen(true);
      return;
    }
    if (!autoOpenOnFirstRun) return;
    if (persistenceKey === undefined) {
      setOpen(true);
      return;
    }
    try {
      const seen = window.localStorage.getItem(persistenceKey);
      if (seen !== "true") {
        setOpen(true);
      }
    } catch {
      // Private-mode browsers reject localStorage; default to open.
      setOpen(true);
    }
  }, [replay, autoOpenOnFirstRun, persistenceKey]);

  const next = useCallback(() => {
    setStep((n) => Math.min(n + 1, totalSteps - 1));
  }, [totalSteps]);

  const back = useCallback(() => {
    setStep((n) => Math.max(n - 1, 0));
  }, []);

  const goTo = useCallback(
    (target: number) => {
      const clamped = Math.max(0, Math.min(target, totalSteps - 1));
      setStep(clamped);
    },
    [totalSteps],
  );

  const dismiss = useCallback(
    (markSeen = true) => {
      if (markSeen && persistenceKey !== undefined) {
        try {
          window.localStorage.setItem(persistenceKey, "true");
        } catch {
          // ignore
        }
      }
      setOpen(false);
      setStep(0);
    },
    [persistenceKey],
  );

  const open_ = useCallback(() => {
    setStep(0);
    setOpen(true);
  }, []);

  return {
    open,
    step,
    isLast: step === totalSteps - 1,
    next,
    back,
    goTo,
    dismiss,
    open_,
  };
}
