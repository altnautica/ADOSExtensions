import type { ReactNode } from "react";

import { cn } from "../../lib/utils";

/** A titled card grouping one part of a page. */
export function Section({
  title,
  actions,
  className,
  children,
}: {
  title: string;
  actions?: ReactNode;
  className?: string;
  children: ReactNode;
}) {
  return (
    <section
      className={cn(
        "we:flex we:flex-col we:gap-2 we:rounded we:border we:border-border-default we:bg-bg-secondary we:p-3",
        className,
      )}
    >
      <div className="we:flex we:items-center we:gap-2">
        <h3 className="we:text-[11px] we:font-medium we:uppercase we:tracking-wide we:text-text-tertiary">{title}</h3>
        {actions && <div className="we:ml-auto we:flex we:items-center we:gap-1">{actions}</div>}
      </div>
      {children}
    </section>
  );
}
