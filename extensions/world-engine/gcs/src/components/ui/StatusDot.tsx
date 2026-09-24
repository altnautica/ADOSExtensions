import { cn } from "../../lib/utils";

/** The node/health status vocabulary. Colour is never the only channel. */
export type StatusLevel = "good" | "warning" | "serious" | "critical" | "idle" | "offline";

const FILL: Record<StatusLevel, string> = {
  good: "we:bg-status-success",
  warning: "we:bg-status-warning",
  serious: "we:bg-status-serious",
  critical: "we:bg-status-error",
  idle: "we:bg-accent-primary",
  offline: "we:bg-text-tertiary",
};

const RING: Record<StatusLevel, string> = {
  good: "we:border-status-success",
  warning: "we:border-status-warning",
  serious: "we:border-status-serious",
  critical: "we:border-status-error",
  idle: "we:border-accent-primary",
  offline: "we:border-text-tertiary",
};

const SIZE = { xs: "we:w-1.5 we:h-1.5", sm: "we:w-2 we:h-2", md: "we:w-2.5 we:h-2.5" } as const;

export function StatusDot({
  status,
  shape = "dot",
  pulse = false,
  label,
  size = "sm",
  className,
}: {
  status: StatusLevel;
  /** "dot" = filled; "ring" = hollow, for unknown / standby. */
  shape?: "dot" | "ring";
  pulse?: boolean;
  /** Accessible label and tooltip; defaults to the status word. */
  label?: string;
  size?: keyof typeof SIZE;
  className?: string;
}) {
  return (
    <span
      role="img"
      aria-label={label ?? status}
      title={label ?? status}
      className={cn(
        "we:inline-block we:flex-shrink-0 we:rounded-full",
        SIZE[size],
        shape === "ring" ? cn("we:border we:bg-transparent", RING[status]) : FILL[status],
        pulse && "we:animate-pulse",
        className,
      )}
    />
  );
}
