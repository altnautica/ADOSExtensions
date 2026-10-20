/**
 * @module sessions/SessionTable
 * @description The device's recorded sessions: duration, size, rooms, the
 * upload progress (bar, rate and ETA while uploading), and the per-row
 * upload and delete actions. Clicking a row selects it.
 */

import { CloudUpload, Trash2 } from "lucide-react";

import { translator } from "../../host/i18n";
import type { CaptureClient } from "../../lib/agent/capture-client";
import {
  formatBytes,
  formatDuration,
  uploadEtaS,
  uploadFraction,
  type CaptureSession,
} from "../../lib/capture/sessions";
import { cn } from "../../lib/utils";
import { Button } from "../ui/Button";
import { useToast } from "../ui/Toast";

const t = translator("sessions");

export function UploadCell({ session, rate }: { session: CaptureSession; rate: number | null }) {
  const u = session.upload;
  const f = uploadFraction(u);
  const eta = uploadEtaS(u, rate);
  return (
    <div className="we:flex we:min-w-32 we:flex-col we:gap-0.5" data-upload={u.state}>
      <span className={cn(u.state === "failed" || u.state === "blocked" ? "we:text-status-error" : "we:text-text-secondary")}>
        {t(`upload.${u.state}`)}
        {f !== null && u.state !== "idle" && u.state !== "blocked" && ` ${Math.round(f * 100)} %`}
      </span>
      {(u.state === "uploading" || u.state === "queued") && (
        <div className="we:h-1 we:rounded we:bg-bg-tertiary" role="progressbar" aria-valuenow={f === null ? undefined : Math.round(f * 100)} aria-valuemin={0} aria-valuemax={100}>
          <div className="we:h-1 we:rounded we:bg-accent-primary" style={{ width: `${(f ?? 0) * 100}%` }} />
        </div>
      )}
      {u.state === "uploading" && rate !== null && (
        <span className="we:font-mono we:text-[10px] we:text-text-tertiary">
          {formatBytes(rate)}/s{eta !== null && ` · ${formatDuration(eta)}`}
        </span>
      )}
      {u.error && <span className="we:text-[10px] we:text-status-error">{u.error}</span>}
    </div>
  );
}

export function SessionTable({
  client,
  sessions,
  rates,
  selectedId,
  onSelect,
  onChanged,
}: {
  client: CaptureClient;
  sessions: readonly CaptureSession[];
  rates: Record<string, number>;
  selectedId: string | null;
  onSelect: (id: string) => void;
  onChanged: () => void;
}) {
  const { toast } = useToast();

  const upload = async (id: string) => {
    const res = await client.upload(id);
    if (!res.ok) toast(res.code === "transport_error" ? t("errUnreachable") : res.message, "error");
    onChanged();
  };
  const remove = async (s: CaptureSession) => {
    if (!s.uploaded && !window.confirm(t("deleteUnuploaded"))) return;
    const res = await client.deleteSession(s.sessionId, !s.uploaded);
    if (!res.ok) toast(res.message, "error");
    onChanged();
  };

  return (
    <table className="we:w-full we:text-[11px]">
      <thead className="we:text-left we:text-text-tertiary">
        <tr>
          <th className="we:py-1 we:font-medium">{t("col.session")}</th>
          <th className="we:font-medium">{t("col.duration")}</th>
          <th className="we:font-medium">{t("col.size")}</th>
          <th className="we:font-medium">{t("col.rooms")}</th>
          <th className="we:font-medium">{t("col.upload")}</th>
          <th />
        </tr>
      </thead>
      <tbody>
        {sessions.map((s) => (
          <tr
            key={s.sessionId}
            onClick={() => onSelect(s.sessionId)}
            aria-selected={s.sessionId === selectedId}
            className={cn(
              "we:cursor-pointer we:border-t we:border-border-default",
              s.sessionId === selectedId && "we:bg-accent-primary/10",
            )}
          >
            <td className="we:py-1.5 we:font-mono we:text-text-secondary">{s.sessionId}</td>
            <td className="we:font-mono">{formatDuration(s.durationS)}</td>
            <td className="we:font-mono">{formatBytes(s.sizeBytes)}</td>
            <td className="we:max-w-40 we:truncate" title={s.rooms.join(", ")}>
              {s.rooms.length > 0 ? s.rooms.join(", ") : "–"}
            </td>
            <td>
              <UploadCell session={s} rate={rates[s.sessionId] ?? null} />
            </td>
            <td className="we:whitespace-nowrap we:text-right" onClick={(e) => e.stopPropagation()}>
              {!s.uploaded && (s.upload.state === "idle" || s.upload.state === "failed") && (
                <Button size="sm" variant="ghost" aria-label={t("upload.start")} icon={<CloudUpload size={12} />} onClick={() => void upload(s.sessionId)} />
              )}
              <Button size="sm" variant="ghost" aria-label={t("delete")} icon={<Trash2 size={12} />} onClick={() => void remove(s)} />
            </td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}
