import { createContext, useCallback, useContext, useMemo, useRef, useState, type ReactNode } from "react";

import { cn } from "../../lib/utils";

export type ToastKind = "info" | "success" | "warning" | "error";

interface ToastItem {
  id: number;
  message: string;
  kind: ToastKind;
}

interface ToastApi {
  toast: (message: string, kind?: ToastKind) => void;
}

/** How long a toast stays up. */
const TOAST_MS = 4000;

const KIND: Record<ToastKind, string> = {
  info: "we:border-border-strong",
  success: "we:border-status-success/60",
  warning: "we:border-status-warning/60",
  error: "we:border-status-error/60",
};

const ToastContext = createContext<ToastApi | null>(null);

/** Short-lived notices for the page's own actions, stacked in its corner. */
export function ToastProvider({ children }: { children: ReactNode }) {
  const [items, setItems] = useState<ToastItem[]>([]);
  const nextId = useRef(0);
  const toast = useCallback((message: string, kind: ToastKind = "info") => {
    const id = nextId.current++;
    setItems((list) => [...list, { id, message, kind }]);
    setTimeout(() => setItems((list) => list.filter((t) => t.id !== id)), TOAST_MS);
  }, []);
  const api = useMemo(() => ({ toast }), [toast]);
  return (
    <ToastContext.Provider value={api}>
      {children}
      <div
        aria-live="polite"
        className="we:pointer-events-none we:absolute we:bottom-3 we:right-3 we:z-50 we:flex we:flex-col we:gap-2"
      >
        {items.map((t) => (
          <div
            key={t.id}
            role={t.kind === "error" ? "alert" : "status"}
            className={cn(
              "we:max-w-xs we:rounded we:border we:bg-bg-secondary we:px-3 we:py-2 we:text-xs we:text-text-primary we:shadow-lg",
              KIND[t.kind],
            )}
          >
            {t.message}
          </div>
        ))}
      </div>
    </ToastContext.Provider>
  );
}

export function useToast(): ToastApi {
  const api = useContext(ToastContext);
  if (!api) throw new Error("useToast outside ToastProvider");
  return api;
}
