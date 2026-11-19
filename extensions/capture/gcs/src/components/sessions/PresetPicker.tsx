/**
 * @module sessions/PresetPicker
 * @description The reconstruction preset radio group (Preview / HQ / Tour /
 * Warehouse) and the Reconstruct button. Picking a preset and pressing
 * Reconstruct asks the device to forward the session to its paired node;
 * the reply names the node and job so the page can follow the pipeline.
 */

import { useId, useState } from "react";
import { Hammer } from "lucide-react";

import { translator } from "../../host/i18n";
import type { CaptureClient } from "../../lib/agent/capture-client";
import { RECONSTRUCT_PRESETS, type ReconstructPreset } from "../../lib/capture/sessions";
import { cn } from "@altnautica/mapping-ui/lib/utils";
import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { useToast } from "@altnautica/mapping-ui/components/ui/Toast";

const t = translator("sessions");

const ERROR_KEYS: Record<string, string> = {
  E_NOT_UPLOADED: "errNotUploaded",
  E_TOOL_MISSING: "errToolMissing",
  transport_error: "errUnreachable",
  E_CAPTURE_UNAVAILABLE: "errServiceDown",
};

export function PresetPicker({
  client,
  sessionId,
  disabled,
  onStarted,
}: {
  client: CaptureClient;
  sessionId: string;
  disabled?: boolean;
  onStarted: (job: { node: string | null; jobId: string | null; preset: ReconstructPreset }) => void;
}) {
  const name = useId();
  const { toast } = useToast();
  const [preset, setPreset] = useState<ReconstructPreset>("preview");
  const [busy, setBusy] = useState(false);

  const reconstruct = async () => {
    setBusy(true);
    const res = await client.reconstruct(sessionId, preset);
    setBusy(false);
    if (res.ok) {
      toast(t("queued", { preset: t(`preset.${preset}`) }), "success");
      onStarted({ ...res.value, preset });
    } else {
      toast(ERROR_KEYS[res.code] ? t(ERROR_KEYS[res.code]!) : res.message, "error");
    }
  };

  return (
    <div className="cap:flex cap:flex-wrap cap:items-center cap:gap-2">
      <div role="radiogroup" aria-label={t("presetLabel")} className="cap:flex cap:gap-1">
        {RECONSTRUCT_PRESETS.map((p) => (
          <label
            key={p}
            title={t(`presetHint.${p}`)}
            className={cn(
              "cap:cursor-pointer cap:rounded cap:px-2 cap:py-1 cap:text-[11px] cap:border",
              preset === p
                ? "cap:border-accent-primary cap:bg-accent-primary/15 cap:text-accent-primary"
                : "cap:border-border-default cap:text-text-secondary",
            )}
          >
            <input
              type="radio"
              name={name}
              value={p}
              checked={preset === p}
              onChange={() => setPreset(p)}
              className="cap:sr-only"
            />
            {t(`preset.${p}`)}
          </label>
        ))}
      </div>
      <Button icon={<Hammer size={12} />} loading={busy} disabled={disabled} onClick={() => void reconstruct()}>
        {t("reconstruct")}
      </Button>
    </div>
  );
}
