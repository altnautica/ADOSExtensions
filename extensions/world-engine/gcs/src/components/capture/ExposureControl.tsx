/**
 * @module capture/ExposureControl
 * @description Exposure lock: the current exposure and gain, the capture
 * mode (walking ≤ 4 ms, slow ≤ 10 ms, short ≤ 2 ms against mains flicker) and
 * a lock / unlock toggle. A locked exposure keeps one segment consistent for
 * the trainer; a lock the cameras did not take is flagged with its reason.
 */

import { useState } from "react";
import { Lock, Unlock } from "lucide-react";

import { translator } from "../../host/i18n";
import type { CaptureClient } from "../../lib/agent/capture-client";
import type { ExposureMode, ExposureState } from "../../lib/capture/capture-state";
import { Button } from "../ui/Button";
import { useToast } from "../ui/Toast";

const t = translator("capture");
const MODES: readonly ExposureMode[] = ["walking", "slow", "short"];

export function ExposureControl({
  client,
  exposure,
  onChanged,
}: {
  client: CaptureClient;
  exposure: ExposureState;
  onChanged: () => void;
}) {
  const { toast } = useToast();
  const [busy, setBusy] = useState(false);

  const apply = async (locked: boolean, mode: ExposureMode | undefined) => {
    setBusy(true);
    const res = await client.setExposure({
      locked,
      mode,
      exposureUs: exposure.exposureUs ?? undefined,
      gain: exposure.gain ?? undefined,
    });
    setBusy(false);
    if (!res.ok) toast(res.message, "error");
    onChanged();
  };

  return (
    <div className="we:flex we:flex-wrap we:items-center we:gap-2 we:text-[11px] we:text-text-secondary">
      <span className="we:font-mono">
        {exposure.exposureUs !== null ? `${(exposure.exposureUs / 1000).toFixed(1)} ms` : "–"}
        {" · "}
        {exposure.gain !== null ? `${exposure.gain.toFixed(1)}×` : "–"}
      </span>
      <span className={exposure.locked ? "we:text-status-success" : "we:text-text-tertiary"}>
        {exposure.locked ? t("exposureLocked") : t("exposureAuto")}
      </span>
      {exposure.locked && !exposure.applied && (
        <span className="we:text-status-warning" role="status">
          {exposure.reason === "pending"
            ? t("exposureApplying")
            : exposure.reason
              ? t("exposureNotAppliedReason", { reason: exposure.reason })
              : t("exposureNotApplied")}
        </span>
      )}
      <select
        aria-label={t("exposureMode")}
        value={exposure.mode ?? "walking"}
        disabled={busy}
        onChange={(e) => void apply(exposure.locked, e.target.value as ExposureMode)}
        className="we:h-7 we:rounded we:border we:border-border-default we:bg-bg-primary we:px-1 we:text-xs"
      >
        {MODES.map((m) => (
          <option key={m} value={m}>
            {t(`exposureModes.${m}`)}
          </option>
        ))}
      </select>
      <Button
        size="sm"
        variant="secondary"
        loading={busy}
        icon={exposure.locked ? <Unlock size={12} /> : <Lock size={12} />}
        onClick={() => void apply(!exposure.locked, exposure.mode ?? undefined)}
      >
        {exposure.locked ? t("unlock") : t("lock")}
      </Button>
    </div>
  );
}
