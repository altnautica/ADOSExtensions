import { Loader2 } from "lucide-react";
import type { ButtonHTMLAttributes, ReactNode } from "react";

import { cn } from "../../lib/utils";

interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: "primary" | "secondary" | "ghost" | "danger";
  size?: "sm" | "md" | "lg";
  icon?: ReactNode;
  loading?: boolean;
}

const VARIANT: Record<NonNullable<ButtonProps["variant"]>, string> = {
  primary: "map:bg-accent-primary map:text-accent-foreground map:hover:bg-accent-primary-hover",
  secondary:
    "map:bg-bg-tertiary map:text-text-primary map:border map:border-border-default map:hover:border-border-strong",
  ghost: "map:text-text-secondary map:hover:text-text-primary map:hover:bg-bg-tertiary",
  danger:
    "map:bg-status-error/20 map:text-status-error map:border map:border-status-error/30 map:hover:bg-status-error/30",
};

const SIZE: Record<NonNullable<ButtonProps["size"]>, string> = {
  sm: "map:h-7 map:px-2.5 map:text-xs map:gap-1.5",
  md: "map:h-8 map:px-3 map:text-xs map:gap-2",
  lg: "map:h-10 map:px-4 map:text-sm map:gap-2",
};

export function Button({
  variant = "primary",
  size = "md",
  icon,
  loading,
  disabled,
  className,
  type = "button",
  children,
  ...props
}: ButtonProps) {
  return (
    <button
      type={type}
      className={cn(
        "map:inline-flex map:items-center map:justify-center map:font-medium map:transition-colors map:cursor-pointer",
        "map:disabled:opacity-50 map:disabled:cursor-not-allowed",
        "map:focus-visible:outline-2 map:focus-visible:outline-accent-primary",
        // A touch-sized target wherever the primary pointer is coarse.
        "map:pointer-coarse:min-h-12 map:pointer-coarse:min-w-12",
        VARIANT[variant],
        SIZE[size],
        className,
      )}
      disabled={disabled || loading}
      aria-busy={loading || undefined}
      {...props}
    >
      {loading ? <Loader2 size={14} className="map:animate-spin" /> : icon}
      {children}
    </button>
  );
}
