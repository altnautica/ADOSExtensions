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
    <label className={cn("map:flex map:flex-col map:gap-1", className)}>
      {label && <span className="map:text-[11px] map:text-text-secondary">{label}</span>}
      <span className="map:relative map:flex map:items-center">
        <select
          value={value}
          disabled={disabled}
          onChange={(e) => onChange(e.target.value)}
          className={cn(
            "map:h-8 map:w-full map:appearance-none map:rounded map:border map:border-border-default map:bg-bg-tertiary",
            "map:pl-2.5 map:pr-7 map:text-xs map:text-text-primary map:outline-none map:focus:border-accent-primary",
            "map:disabled:cursor-not-allowed map:disabled:opacity-50",
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
          className="map:pointer-events-none map:absolute map:right-2 map:text-text-tertiary"
        />
      </span>
    </label>
  );
}
