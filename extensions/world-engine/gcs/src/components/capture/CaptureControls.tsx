/**
 * @module capture/CaptureControls
 * @description REC / STOP, MARK with the room name and the floor stepper,
 * the "return to start" prompt and the storage left. MARK starts a new
 * segment named after the room; changing floor while recording marks a
 * transition segment on the new floor.
 */

import { useState } from "react";
import { Circle, CornerDownLeft, Flag, Minus, Plus, Square } from "lucide-react";

import { translator } from "../../host/i18n";
import type { ActionResult, CaptureClient } from "../../lib/agent/capture-client";
import { returnToStartPrompt, type CaptureCoverage, type CaptureState } from "../../lib/capture/capture-state";
import { formatBytes, formatDuration } from "../../lib/capture/sessions";
import { Button } from "../ui/Button";
import { useToast } from "../ui/Toast";

const t = translator("capture");

/** Error codes the capture surface answers with, mapped to their messages. */
const ERROR_KEYS: Record<string, string> = {
  E_STORAGE_FULL: "errStorageFull",
  E_ALREADY_RECORDING: "errAlreadyRecording",
  service_unavailable: "errServiceDown",
  transport_error: "errUnreachable",
};

export function CaptureControls({
  client,
  state,
  coverage,
  onChanged,
}: {
  client: CaptureClient;
  state: CaptureState;
  coverage: CaptureCoverage | null;
  onChanged: () => void;
}) {
  const { toast } = useToast();
  const [room, setRoom] = useState(state.room ?? "");
  const [floor, setFloor] = useState(state.floor);
  const [busy, setBusy] = useState(false);
  const recording = state.state === "recording" || state.state === "paused";
  const prompt = returnToStartPrompt(state, coverage);

  const run = async <T,>(action: () => Promise<ActionResult<T>>, okKey: string) => {
    setBusy(true);
    const res = await action();
    setBusy(false);
    if (res.ok) toast(t(okKey), "success");
    else toast(ERROR_KEYS[res.code] ? t(ERROR_KEYS[res.code]!) : res.message, "error");
    onChanged();
  };

  const stepFloor = (delta: number) => {
    const next = floor + delta;
    setFloor(next);
    if (recording) void run(() => client.mark(room.trim() || t("stairs"), next), "marked");
  };

  return (
    <div className="we:flex we:flex-col we:gap-2">
      <div className="we:flex we:flex-wrap we:items-center we:gap-2">
        {recording ? (
          <Button variant="danger" icon={<Square size={12} />} loading={busy} onClick={() => void run(() => client.stop(), "stopped")}>
            {t("stop")}
          </Button>
        ) : (
          <Button
            icon={<Circle size={12} />}
            loading={busy}
            disabled={state.state === "closing"}
            onClick={() => void run(() => client.start({ room: room.trim() || undefined, floor }), "started")}
          >
            {t("rec")}
          </Button>
        )}
        <Button
          variant="secondary"
          icon={<Flag size={12} />}
          disabled={!recording || busy || room.trim() === ""}
          onClick={() => void run(() => client.mark(room.trim(), floor), "marked")}
        >
          {t("mark")}
        </Button>
        <label className="we:flex we:items-center we:gap-1 we:text-[11px] we:text-text-tertiary">
          {t("roomName")}
          <input
            value={room}
            onChange={(e) => setRoom(e.target.value)}
            maxLength={48}
            className="we:h-7 we:w-36 we:rounded we:border we:border-border-default we:bg-bg-primary we:px-2 we:text-xs we:text-text-primary"
          />
        </label>
        <div className="we:flex we:items-center we:gap-1 we:text-[11px] we:text-text-tertiary" role="group" aria-label={t("floor")}>
          {t("floor")}
          <Button size="sm" variant="ghost" aria-label={t("floorDown")} onClick={() => stepFloor(-1)} icon={<Minus size={12} />} />
          <span className="we:w-6 we:text-center we:font-mono we:text-text-primary" aria-live="polite">
            {floor}
          </span>
          <Button size="sm" variant="ghost" aria-label={t("floorUp")} onClick={() => stepFloor(1)} icon={<Plus size={12} />} />
        </div>
      </div>
      <div className="we:flex we:flex-wrap we:items-center we:gap-3 we:text-[11px] we:text-text-tertiary">
        {recording && (
          <span className="we:font-mono we:text-status-error">
            ● {formatDuration(state.durationS)}
            {state.segment !== null && ` · ${t("segment", { n: state.segment })}`}
            {state.room && ` · ${state.room}`}
          </span>
        )}
        <span>
          {t("storageLeft", {
            free: formatBytes(state.storage.freeBytes),
            time: formatDuration(state.storage.estRemainingS),
          })}
        </span>
        {prompt.kind === "return" && (
          <span className="we:flex we:items-center we:gap-1 we:text-status-warning">
            <CornerDownLeft size={12} />
            {prompt.bearingDeg === null
              ? t("returnToStart", { d: prompt.distanceM.toFixed(0) })
              : t("returnToStartBearing", {
                  d: prompt.distanceM.toFixed(0),
                  b: prompt.bearingDeg.toFixed(0),
                  turn:
                    prompt.relativeDeg === null
                      ? ""
                      : t(Math.abs(prompt.relativeDeg) < 15 ? "ahead" : prompt.relativeDeg > 0 ? "turnRight" : "turnLeft", {
                          deg: Math.abs(prompt.relativeDeg).toFixed(0),
                        }),
                })}
          </span>
        )}
        {prompt.kind === "close_loop" && (
          <span className="we:flex we:items-center we:gap-1 we:text-status-success">
            <CornerDownLeft size={12} />
            {t("closeLoop")}
          </span>
        )}
      </div>
    </div>
  );
}
