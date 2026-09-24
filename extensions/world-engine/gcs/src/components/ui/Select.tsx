import { ChevronDown } from "lucide-react";
import type { ReactNode } from "react";

import { cn } from "../../lib/utils";

export interface SelectOption {
  value: string;
  label: string;
}

interface SelectProps {
  label?: ReactNode;
  options: readonly SelectOption[];
  value: string;
  onChange: (value: string) => void;
  /** Shown when `value` matches no option. */
  placeholder?: string;
  disabled?: boolean;
  className?: string;
}

/** A styled native select: keyboard, screen-reader and touch behaviour come
 * from the platform. A value no option carries renders as the placeholder, or
 * as itself when there is no placeholder, so a stored value is never hidden. */
export function Select({ label, options, value, onChange, placeholder, disabled, className }: SelectProps) {
  const known = options.some((o) => o.value === value);
  return (
    <label className={cn("we:flex we:flex-col we:gap-1", className)}>
      {label && <span className="we:text-[11px] we:text-text-secondary">{label}</span>}
      <span className="we:relative we:flex we:items-center">
        <select
          value={value}
          disabled={disabled}
          onChange={(e) => onChange(e.target.value)}
          className={cn(
            "we:h-8 we:w-full we:appearance-none we:rounded we:border we:border-border-default we:bg-bg-tertiary",
            "we:pl-2.5 we:pr-7 we:text-xs we:text-text-primary we:outline-none we:focus:border-accent-primary",
            "we:disabled:cursor-not-allowed we:disabled:opacity-50",
          )}
        >
          {!known && (
            <option value={value} disabled={placeholder !== undefined}>
              {placeholder ?? value}
            </option>
          )}
          {options.map((o) => (
            <option key={o.value} value={o.value}>
              {o.label}
            </option>
          ))}
        </select>
        <ChevronDown
          size={12}
          aria-hidden
          className="we:pointer-events-none we:absolute we:right-2 we:text-text-tertiary"
        />
      </span>
    </label>
  );
}
