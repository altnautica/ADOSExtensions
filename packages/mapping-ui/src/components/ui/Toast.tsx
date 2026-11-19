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
  info: "map:border-border-strong",
  success: "map:border-status-success/60",
  warning: "map:border-status-warning/60",
  error: "map:border-status-error/60",
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
        className="map:pointer-events-none map:absolute map:bottom-3 map:right-3 map:z-50 map:flex map:flex-col map:gap-2"
      >
        {items.map((t) => (
          <div
            key={t.id}
            role={t.kind === "error" ? "alert" : "status"}
            className={cn(
              "map:max-w-xs map:rounded map:border map:bg-bg-secondary map:px-3 map:py-2 map:text-xs map:text-text-primary map:shadow-lg",
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
