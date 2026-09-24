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
  primary: "we:bg-accent-primary we:text-accent-foreground we:hover:bg-accent-primary-hover",
  secondary:
    "we:bg-bg-tertiary we:text-text-primary we:border we:border-border-default we:hover:border-border-strong",
  ghost: "we:text-text-secondary we:hover:text-text-primary we:hover:bg-bg-tertiary",
  danger:
    "we:bg-status-error/20 we:text-status-error we:border we:border-status-error/30 we:hover:bg-status-error/30",
};

const SIZE: Record<NonNullable<ButtonProps["size"]>, string> = {
  sm: "we:h-7 we:px-2.5 we:text-xs we:gap-1.5",
  md: "we:h-8 we:px-3 we:text-xs we:gap-2",
  lg: "we:h-10 we:px-4 we:text-sm we:gap-2",
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
        "we:inline-flex we:items-center we:justify-center we:font-medium we:transition-colors we:cursor-pointer",
        "we:disabled:opacity-50 we:disabled:cursor-not-allowed",
        "we:focus-visible:outline-2 we:focus-visible:outline-accent-primary",
        VARIANT[variant],
        SIZE[size],
        className,
      )}
      disabled={disabled || loading}
      aria-busy={loading || undefined}
      {...props}
    >
      {loading ? <Loader2 size={14} className="we:animate-spin" /> : icon}
      {children}
    </button>
  );
}
