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
        "map:flex map:flex-col map:gap-2 map:rounded map:border map:border-border-default map:bg-bg-secondary map:p-3",
        className,
      )}
    >
      <div className="map:flex map:items-center map:gap-2">
        <h3 className="map:text-[11px] map:font-medium map:uppercase map:tracking-wide map:text-text-tertiary">{title}</h3>
        {actions && <div className="map:ml-auto map:flex map:items-center map:gap-1">{actions}</div>}
      </div>
      {children}
    </section>
  );
}
