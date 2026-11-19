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
import { StatusDot } from "@altnautica/mapping-ui/components/ui/StatusDot";

const t = translator("capture");

export function HealthChips({ signals }: { signals: readonly HealthSignal[] }) {
  const chips = healthChips(signals);
  const prompts = chips.flatMap((c) => (c.promptKey ? [c] : []));
  return (
    <div className="cap:flex cap:flex-col cap:gap-1.5">
      <ul className="cap:flex cap:flex-wrap cap:gap-1.5" aria-label={t("healthLabel")}>
        {chips.map((c) => (
          <li
            key={c.id}
            data-chip={c.id}
            data-level={c.level ?? "unknown"}
            className="cap:flex cap:items-center cap:gap-1.5 cap:rounded-full cap:bg-bg-tertiary cap:px-2 cap:py-0.5 cap:text-[11px] cap:text-text-secondary"
          >
            <StatusDot status={c.tone} label={t(`level.${c.level ?? "unknown"}`)} size="xs" />
            <span>{t(`health.${c.labelKey}`)}</span>
            {c.reading && <span className="cap:font-mono cap:text-text-tertiary">{c.reading}</span>}
          </li>
        ))}
      </ul>
      {prompts.map((c) => (
        <p key={c.id} role="alert" className="cap:flex cap:items-center cap:gap-1.5 cap:text-[11px] cap:text-status-error">
          <AlertTriangle size={12} />
          {t(`prompt.${c.promptKey}`)}
        </p>
      ))}
    </div>
  );
}
