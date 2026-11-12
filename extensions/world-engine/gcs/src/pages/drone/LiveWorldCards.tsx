/**
 * The Live World capture and stream cards: keyframe / ingest / camera counts,
 * tracking health, pose source and dropped keyframes, plus the paired
 * reconstructor, the active bearer and the last keyframe's age, from the
 * drone's live capture slice. A capture that builds no world model (a link
 * that carries no keyframes, no position anchor yet, the keyframe cap
 * reached) says so. Unreported values render as the no-data glyph.
 */

import type { ReactNode } from "react";
import { Activity, Radio, TriangleAlert } from "lucide-react";

import { translator } from "../../host/i18n";
import { bearerWordKey, degradedReasonKey, healthWordKey, poseTierKey } from "../../lib/atlas/live-words";
import { nodeLabelForId, type NamedNode } from "../../lib/nodes/node-label";
import type { AtlasLiveState } from "../../stores/atlas-store";
import { NO_DATA_GLYPH, cn } from "../../lib/utils";

const t = translator("atlas");

const VIO_TONE: Record<string, string> = {
  good: "we:text-status-success",
  degraded: "we:text-status-warning",
  lost: "we:text-status-error",
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

function Row({ label, children, title, tone }: { label: string; children: ReactNode; title?: string; tone?: string }) {
  return (
    <div className="we:flex we:items-center we:justify-between">
      <span className="we:text-text-tertiary">{label}</span>
      <span
        className={cn("we:max-w-[60%] we:truncate we:font-mono we:tabular-nums", tone ?? "we:text-text-secondary")}
        title={title}
      >
        {children}
      </span>
    </div>
  );
}

/** A capture condition under which no world model is being built. */
function Notice({ children }: { children: ReactNode }) {
  return (
    <div
      role="status"
      className="we:flex we:items-start we:gap-1.5 we:rounded we:bg-status-warning/10 we:px-2 we:py-1.5 we:text-[11px] we:text-status-warning"
    >
      <TriangleAlert className="we:mt-0.5 we:h-3 we:w-3 we:shrink-0" />
      <div>{children}</div>
    </div>
  );
}

export function LiveWorldCards({
  live,
  stale,
  lastKeyframeAgo,
  nodes,
}: {
  live: AtlasLiveState;
  /** The heartbeat went quiet past its budget: the rates are dimmed. */
  stale: boolean;
  lastKeyframeAgo: string | null;
  /** The paired nodes, to name the compute node. */
  nodes: readonly NamedNode[];
}) {
  const num = (v: number | null) => (v === null ? NO_DATA_GLYPH : String(v));
  const computeNode = live.computeNodeId === null ? null : nodeLabelForId(live.computeNodeId, nodes);
  const reasonKey = degradedReasonKey(live.bearer);
  const degradedReason = reasonKey ? t(reasonKey) : live.degradedReason;
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
        {live.anchored === false && <Notice>{t("capture.waitingForAnchor")}</Notice>}
        {live.capped === true && <Notice>{t("capture.cappedNotice")}</Notice>}
        <div className="we:space-y-1.5 we:text-[11px]">
          {live.vioHealth !== null && (
            <Row label={t("capture.vioTracking")} tone={VIO_TONE[live.vioHealth]}>
              {t(healthWordKey(live.vioHealth))}
            </Row>
          )}
          {live.poseTier !== null && <Row label={t("capture.poseTierLabel")}>{t(poseTierKey(live.poseTier))}</Row>}
          {live.droppedKeyframes !== null && (
            <Row
              label={t("capture.droppedKeyframes")}
              tone={live.droppedKeyframes > 0 ? "we:text-status-warning" : undefined}
            >
              {String(live.droppedKeyframes)}
            </Row>
          )}
        </div>
      </Card>

      <Card title={t("capture.streamSection")} icon={Radio}>
        <div className="we:space-y-1.5 we:text-[11px]">
          <Row label={t("capture.computeNode")} title={computeNode?.title}>
            {computeNode?.label ?? NO_DATA_GLYPH}
          </Row>
          <Row label={t("capture.bearer")}>{live.bearer ? t(bearerWordKey(live.bearer)) : NO_DATA_GLYPH}</Row>
          <Row label={t("capture.lastKeyframe")}>{lastKeyframeAgo ?? NO_DATA_GLYPH}</Row>
        </div>
        {live.keyframesCarried === false && (
          <Notice>
            <span className="we:block we:font-medium">{t("capture.noKeyframesOnLink")}</span>
            {degradedReason && <span className="we:block">{degradedReason}</span>}
          </Notice>
        )}
      </Card>
    </div>
  );
}
