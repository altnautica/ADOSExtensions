/**
 * @module SegmentedPane
 * @description Several views of one subsystem behind a segmented control
 * (a tablist with arrow-key, Home and End navigation), rendering only the
 * active segment.
 */

import { useId, useState, type ReactNode } from "react";

import { cn } from "../../lib/utils";

export interface PaneSegment {
  id: string;
  label: string;
  render: () => ReactNode;
}

export function SegmentedPane({
  segments,
  initialId,
  ariaLabel,
}: {
  segments: readonly PaneSegment[];
  /** Which segment opens first; the first segment when absent. */
  initialId?: string;
  ariaLabel: string;
}) {
  const baseId = useId();
  const [activeId, setActiveId] = useState(() => initialId ?? segments[0]?.id ?? "");
  const active = segments.find((s) => s.id === activeId) ?? segments[0];
  if (!active) return null;

  const tabId = (id: string) => `${baseId}-tab-${id}`;
  const panelId = (id: string) => `${baseId}-panel-${id}`;

  const moveTo = (index: number) => {
    const next = segments[index];
    if (!next) return;
    setActiveId(next.id);
    requestAnimationFrame(() => document.getElementById(tabId(next.id))?.focus());
  };

  return (
    <div className="map:flex map:flex-1 map:min-h-0 map:flex-col">
      <div
        role="tablist"
        aria-label={ariaLabel}
        className="map:flex map:shrink-0 map:items-center map:gap-1 map:border-b map:border-border-default map:px-3 map:py-1.5"
      >
        {segments.map((segment, idx) => {
          const selected = segment.id === active.id;
          return (
            <button
              key={segment.id}
              type="button"
              role="tab"
              id={tabId(segment.id)}
              aria-selected={selected}
              aria-controls={panelId(segment.id)}
              tabIndex={selected ? 0 : -1}
              onClick={() => setActiveId(segment.id)}
              onKeyDown={(e) => {
                const last = segments.length - 1;
                if (e.key === "ArrowRight") moveTo(idx === last ? 0 : idx + 1);
                else if (e.key === "ArrowLeft") moveTo(idx === 0 ? last : idx - 1);
                else if (e.key === "Home") moveTo(0);
                else if (e.key === "End") moveTo(last);
                else return;
                e.preventDefault();
              }}
              className={cn(
                "map:rounded map:px-2.5 map:py-1 map:text-xs map:font-medium map:transition-colors map:cursor-pointer",
                "map:focus-visible:outline-2 map:focus-visible:outline-accent-primary",
                selected
                  ? "map:bg-accent-primary/15 map:text-accent-primary"
                  : "map:text-text-secondary map:hover:text-text-primary",
              )}
            >
              {segment.label}
            </button>
          );
        })}
      </div>
      <div
        id={panelId(active.id)}
        role="tabpanel"
        aria-labelledby={tabId(active.id)}
        tabIndex={0}
        className="map:flex map:flex-1 map:min-h-0 map:flex-col map:overflow-y-auto"
      >
        {active.render()}
      </div>
    </div>
  );
}
