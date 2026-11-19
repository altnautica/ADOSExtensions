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
import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { useToast } from "@altnautica/mapping-ui/components/ui/Toast";

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
    <div className="cap:flex cap:flex-wrap cap:items-center cap:gap-2 cap:text-[11px] cap:text-text-secondary">
      <span className="cap:font-mono">
        {exposure.exposureUs !== null ? `${(exposure.exposureUs / 1000).toFixed(1)} ms` : "–"}
        {" · "}
        {exposure.gain !== null ? `${exposure.gain.toFixed(1)}×` : "–"}
      </span>
      <span className={exposure.locked ? "cap:text-status-success" : "cap:text-text-tertiary"}>
        {exposure.locked ? t("exposureLocked") : t("exposureAuto")}
      </span>
      {exposure.locked && !exposure.applied && (
        <span className="cap:text-status-warning" role="status">
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
        className="cap:h-7 cap:rounded cap:border cap:border-border-default cap:bg-bg-primary cap:px-1 cap:text-xs"
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
