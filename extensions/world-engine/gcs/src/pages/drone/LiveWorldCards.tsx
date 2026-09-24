/**
 * The Live World capture and stream cards: keyframe / ingest / camera counts
 * and VIO tracking health, plus the paired reconstructor, the active bearer,
 * relay decimation and the last keyframe's age, from the drone's live capture
 * slice. Unreported values render as the no-data glyph.
 */

import type { ReactNode } from "react";
import { Activity, Radio } from "lucide-react";

import { translator } from "../../host/i18n";
import type { AtlasLiveState } from "../../stores/atlas-store";
import { NO_DATA_GLYPH, cn } from "../../lib/utils";

const t = translator("atlas");

const VIO_TONE: Record<string, string> = {
  good: "we:text-status-success",
  degraded: "we:text-status-warning",
  lost: "we:text-status-error",
};

const BEARER_LABEL_KEY: Record<string, string> = {
  "direct-lan": "capture.bearerDirectLan",
  "wfb-relay": "capture.bearerWfbRelay",
  cloud: "capture.bearerCloud",
};

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="we:rounded we:bg-text-primary/[0.02] we:px-2 we:py-1.5 we:text-center">
      <div className="we:font-mono we:text-sm we:tabular-nums we:text-text-primary">{value}</div>
      <div className="we:text-[9px] we:uppercase we:tracking-wide we:text-text-tertiary">{label}</div>
    </div>
  );
}

function Card({ title, icon: Icon, children }: { title: string; icon: typeof Activity; children: ReactNode }) {
  return (
    <div className="we:space-y-3 we:rounded-lg we:border we:border-border-default we:p-4">
      <div className="we:flex we:items-center we:gap-1.5">
        <Icon className="we:h-3.5 we:w-3.5 we:text-text-tertiary" />
        <span className="we:text-xs we:font-medium we:text-text-secondary">{title}</span>
      </div>
      {children}
    </div>
  );
}

function Row({ label, children, title }: { label: string; children: ReactNode; title?: string }) {
  return (
    <div className="we:flex we:items-center we:justify-between">
      <span className="we:text-text-tertiary">{label}</span>
      <span className="we:max-w-[60%] we:truncate we:font-mono we:tabular-nums we:text-text-secondary" title={title}>
        {children}
      </span>
    </div>
  );
}

export function LiveWorldCards({
  live,
  stale,
  lastKeyframeAgo,
}: {
  live: AtlasLiveState;
  /** The heartbeat went quiet past its budget: the rates are dimmed. */
  stale: boolean;
  lastKeyframeAgo: string | null;
}) {
  const num = (v: number | null) => (v === null ? NO_DATA_GLYPH : String(v));
  const bearerKey = live.bearer ? BEARER_LABEL_KEY[live.bearer] : undefined;
  return (
    <div className={cn("we:grid we:grid-cols-1 we:gap-4 we:transition-opacity we:xl:grid-cols-2", stale && "we:opacity-50")}>
      <Card title={t("capture.captureSection")} icon={Activity}>
        <div className="we:grid we:grid-cols-2 we:gap-2">
          <Stat label={t("capture.statKeyframes")} value={num(live.keyframesIngested)} />
          <Stat
            label={t("capture.statIngestHz")}
            value={live.ingestRateHz === null ? NO_DATA_GLYPH : live.ingestRateHz.toFixed(1)}
          />
          <Stat label={t("capture.statCameras")} value={num(live.cameraCount)} />
        </div>
        {live.vioHealth !== null && (
          <div className="we:flex we:items-center we:justify-between we:text-[11px]">
            <span className="we:text-text-tertiary">{t("capture.vioTracking")}</span>
            <span className={cn("we:font-medium", VIO_TONE[live.vioHealth] ?? "we:text-text-secondary")}>
              {live.vioHealth}
            </span>
          </div>
        )}
      </Card>

      <Card title={t("capture.streamSection")} icon={Radio}>
        <div className="we:space-y-1.5 we:text-[11px]">
          <Row label={t("capture.computeNode")} title={live.computeNodeId ?? undefined}>
            {live.computeNodeId ?? NO_DATA_GLYPH}
          </Row>
          <Row label={t("capture.bearer")}>{bearerKey ? t(bearerKey) : (live.bearer ?? NO_DATA_GLYPH)}</Row>
          {live.bearer === "wfb-relay" && (
            <Row label={t("capture.relayDecimation")}>{num(live.relayDecimation)}</Row>
          )}
          <Row label={t("capture.lastKeyframe")}>{lastKeyframeAgo ?? NO_DATA_GLYPH}</Row>
        </div>
      </Card>
    </div>
  );
}
