/**
 * @module capture/HealthChips
 * @description The six capture health chips (blur, texture, flicker,
 * corridor, storage, clock) with their readings, and the operator prompt of
 * every red chip underneath. Colour is never the only channel: each chip
 * carries its level as text for screen readers.
 */

import { AlertTriangle } from "lucide-react";

import { translator } from "../../host/i18n";
import type { HealthSignal } from "../../lib/capture/capture-state";
import { healthChips } from "../../lib/capture/health-chips";
import { StatusDot } from "../ui/StatusDot";

const t = translator("capture");

export function HealthChips({ signals }: { signals: readonly HealthSignal[] }) {
  const chips = healthChips(signals);
  const prompts = chips.flatMap((c) => (c.promptKey ? [c] : []));
  return (
    <div className="we:flex we:flex-col we:gap-1.5">
      <ul className="we:flex we:flex-wrap we:gap-1.5" aria-label={t("healthLabel")}>
        {chips.map((c) => (
          <li
            key={c.id}
            data-chip={c.id}
            data-level={c.level ?? "unknown"}
            className="we:flex we:items-center we:gap-1.5 we:rounded-full we:bg-bg-tertiary we:px-2 we:py-0.5 we:text-[11px] we:text-text-secondary"
          >
            <StatusDot status={c.tone} label={t(`level.${c.level ?? "unknown"}`)} size="xs" />
            <span>{t(`health.${c.labelKey}`)}</span>
            {c.reading && <span className="we:font-mono we:text-text-tertiary">{c.reading}</span>}
          </li>
        ))}
      </ul>
      {prompts.map((c) => (
        <p key={c.id} role="alert" className="we:flex we:items-center we:gap-1.5 we:text-[11px] we:text-status-error">
          <AlertTriangle size={12} />
          {t(`prompt.${c.promptKey}`)}
        </p>
      ))}
    </div>
  );
}
