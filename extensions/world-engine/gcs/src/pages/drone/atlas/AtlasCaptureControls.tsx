/**
 * The Atlas capture lifecycle bar (Start / Pause / Resume / Stop & Reconstruct)
 * plus an optional "Reconstruct now" action, driven by a `useAtlasControl`
 * result. Only the transitions valid for the reported capture state are shown.
 * A failure toasts: "capture service unavailable" for a 503, a generic message
 * otherwise, never a silent no-op. Shared by the World Model setup view and the
 * Live World page so both drive capture identically.
 */

import { useState } from "react";
import { Boxes, Pause, Play, Square } from "lucide-react";

import { Button } from "../../../components/ui/Button";
import { useToast } from "../../../components/ui/Toast";
import { translator } from "../../../host/i18n";
import type { AtlasControl } from "../../../hooks/use-atlas-control";
import { isActiveCaptureState, type CaptureResult } from "../../../lib/agent/atlas-control-client";

const t = translator("atlas");

/** "Reconstruct now" wiring, resolved by the caller from the paired compute
 * node and the active session. */
export interface ReconstructAction {
  /** Whether the action can run (a reachable compute client and a live session). */
  available: boolean;
  /** i18n key for why it is disabled, or null when available. */
  disabledKey: string | null;
  /** Submit a reconstruct job; resolves true on success. */
  submit: () => Promise<boolean>;
}

export function AtlasCaptureControls({
  control,
  canStart,
  startBlockedKey,
  reconstruct,
  blockedKey = null,
}: {
  control: AtlasControl;
  /** Whether Start is allowed (requirements pass). */
  canStart: boolean;
  /** i18n key for why Start is blocked, or null. */
  startBlockedKey: string | null;
  /** Optional "Reconstruct now" action. */
  reconstruct?: ReconstructAction;
  /** When set, every lifecycle button is disabled with this i18n reason (the
   * drone cannot be commanded from here). */
  blockedKey?: string | null;
}) {
  const { toast } = useToast();
  const [reconstructing, setReconstructing] = useState(false);

  const readiness = control.readiness;
  // An active session follows both the standalone flag and the lifecycle state,
  // so Pause / Resume / Stop stay visible through a paused session even when the
  // agent reports capturing:false with state:"paused".
  const capturing = readiness ? readiness.capturing === true || isActiveCaptureState(readiness.state) : false;
  const paused = readiness?.state === "paused";
  const blocked = blockedKey !== null;
  const blockedTitle = blockedKey ? t(blockedKey) : undefined;

  const handleResult = (result: CaptureResult, failKey: string) => {
    if (result.ok) return;
    if (result.serviceDown) toast(t("capture.captureServiceDown"), "error");
    else if (result.message !== "inactive") toast(t(failKey), "error");
  };

  const onReconstruct = async () => {
    if (!reconstruct?.available) return;
    setReconstructing(true);
    try {
      const ok = await reconstruct.submit();
      toast(ok ? t("capture.reconstructSubmitted") : t("capture.reconstructFailed"), ok ? "success" : "error");
    } finally {
      setReconstructing(false);
    }
  };

  return (
    <div className="we:flex we:flex-wrap we:items-center we:gap-2">
      {!capturing && (
        <Button
          size="sm"
          variant="primary"
          icon={<Play size={12} />}
          loading={control.busy}
          disabled={blocked || !canStart || control.busy}
          title={blocked ? blockedTitle : !canStart && startBlockedKey ? t(startBlockedKey) : undefined}
          onClick={async () => handleResult(await control.start(), "capture.captureStartFailed")}
        >
          {t("capture.startCapture")}
        </Button>
      )}

      {capturing && paused && (
        <Button
          size="sm"
          variant="primary"
          icon={<Play size={12} />}
          loading={control.busy}
          disabled={blocked || control.busy}
          title={blockedTitle}
          onClick={async () => handleResult(await control.resume(), "capture.captureCommandFailed")}
        >
          {t("capture.resumeCapture")}
        </Button>
      )}

      {capturing && !paused && (
        <Button
          size="sm"
          variant="secondary"
          icon={<Pause size={12} />}
          loading={control.busy}
          disabled={blocked || control.busy}
          title={blockedTitle}
          onClick={async () => handleResult(await control.pause(), "capture.captureCommandFailed")}
        >
          {t("capture.pauseCapture")}
        </Button>
      )}

      {capturing && (
        <Button
          size="sm"
          variant="danger"
          icon={<Square size={12} />}
          loading={control.busy}
          disabled={blocked || control.busy}
          title={blockedTitle}
          onClick={async () => handleResult(await control.stop(), "capture.captureStopFailed")}
        >
          {t("capture.stopCapture")}
        </Button>
      )}

      {reconstruct && (
        <Button
          size="sm"
          variant="secondary"
          icon={<Boxes size={12} />}
          loading={reconstructing}
          disabled={!reconstruct.available || reconstructing}
          title={!reconstruct.available && reconstruct.disabledKey ? t(reconstruct.disabledKey) : undefined}
          onClick={() => void onReconstruct()}
        >
          {t("capture.reconstructNow")}
        </Button>
      )}
    </div>
  );
}
