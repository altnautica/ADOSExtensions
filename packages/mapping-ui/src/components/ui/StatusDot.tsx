import { cn } from "../../lib/utils";

/** The node/health status vocabulary. Colour is never the only channel. */
export type StatusLevel = "good" | "warning" | "serious" | "critical" | "idle" | "offline";

const FILL: Record<StatusLevel, string> = {
  good: "map:bg-status-success",
  warning: "map:bg-status-warning",
  serious: "map:bg-status-serious",
  critical: "map:bg-status-error",
  idle: "map:bg-accent-primary",
  offline: "map:bg-text-tertiary",
};

const RING: Record<StatusLevel, string> = {
  good: "map:border-status-success",
  warning: "map:border-status-warning",
  serious: "map:border-status-serious",
  critical: "map:border-status-error",
  idle: "map:border-accent-primary",
  offline: "map:border-text-tertiary",
};

const SIZE = { xs: "map:w-1.5 map:h-1.5", sm: "map:w-2 map:h-2", md: "map:w-2.5 map:h-2.5" } as const;

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
        "map:inline-block map:flex-shrink-0 map:rounded-full",
        SIZE[size],
        shape === "ring" ? cn("map:border map:bg-transparent", RING[status]) : FILL[status],
        pulse && "map:animate-pulse",
        className,
      )}
    />
  );
}
