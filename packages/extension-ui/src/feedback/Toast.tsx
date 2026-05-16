import { useEffect, useState, type CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

interface Props {
  message: string | null;
  kind?: "success" | "warning" | "error" | "info";
  /** Auto-dismiss after this many ms. ``null`` disables auto-dismiss. */
  durationMs?: number | null;
  onDismiss?: () => void;
}

/**
 * Transient floating message. Fixed-positioned bottom-right. Renders
 * nothing when ``message`` is null. Auto-dismisses after
 * ``durationMs`` (default 3500); pass null to disable auto-dismiss.
 */
export function Toast({
  message,
  kind = "info",
  durationMs = 3500,
  onDismiss,
}: Props): JSX.Element | null {
  const [visible, setVisible] = useState<boolean>(message !== null);

  useEffect(() => {
    if (message === null) {
      setVisible(false);
      return;
    }
    setVisible(true);
    if (durationMs === null) return;
    const timer = window.setTimeout(() => {
      setVisible(false);
      if (onDismiss) onDismiss();
    }, durationMs);
    return () => window.clearTimeout(timer);
  }, [message, durationMs, onDismiss]);

  if (!visible || message === null) return null;
  const color = COLORS[kind];
  return (
    <div
      style={toast(color)}
      role="status"
      aria-live="polite"
      data-testid={`ext-ui-toast-${kind}`}
    >
      {message}
    </div>
  );
}

const COLORS: Record<"success" | "warning" | "error" | "info", string> = {
  success: TOKENS.ok,
  warning: TOKENS.warn,
  error: TOKENS.error,
  info: TOKENS.accent,
};

const toast = (color: string): CSSProperties => ({
  position: "fixed",
  right: "1rem",
  bottom: "1rem",
  padding: "0.625rem 1rem",
  background: TOKENS.surface,
  border: `1px solid ${color}`,
  borderLeft: `4px solid ${color}`,
  borderRadius: "0.375rem",
  color: TOKENS.text,
  fontSize: "0.8125rem",
  boxShadow: "0 4px 16px rgba(0,0,0,0.3)",
  zIndex: 60,
  maxWidth: "320px",
});
