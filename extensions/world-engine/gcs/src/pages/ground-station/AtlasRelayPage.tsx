/**
 * @module pages/ground-station/AtlasRelayPage
 * @description Ground-station Atlas relay indicator. Polls the node's relay
 * status through this extension's agent server and, only when a relay is
 * actually running (`up === true`) or has gone quiet (stale), shows the
 * keyframes-seen / forwarded / keep-rate counters plus a staleness badge. A
 * failed read says the node is unreachable (keeping the last snapshot,
 * badged stale), never "no relay". One request is in flight at a time, the
 * next is armed once it settles, and a hidden document skips the read.
 */

import { useEffect, useRef, useState } from "react";
import { Boxes } from "lucide-react";

import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import {
  getAtlasRelayStatus,
  type AtlasRelayRead,
  type AtlasRelayStatus,
} from "../../lib/agent/gs-atlas-relay";
import { cn, NO_DATA_GLYPH } from "../../lib/utils";

const t = translator("atlas");

const POLL_INTERVAL_MS = 2000;
/** Past this since the last snapshot landed here, it is badged stale. Measured
 * on this browser's clock only (receipt age), never against the node's clock. */
const STALE_MS = 15000;

/** The latest read, keyed to the node it came from so a node switch never
 * shows the previous node's relay while the new poll is in flight. The last
 * good snapshot is kept with its receipt time so a failed poll dims it
 * rather than erasing it. */
interface RelayReadState {
  deviceId: string | null;
  last: AtlasRelayRead["kind"] | null;
  snapshot: AtlasRelayStatus | null;
  receivedAt: number;
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="we:rounded we:bg-bg-tertiary we:px-2 we:py-1.5 we:text-center">
      <div className="we:text-sm we:font-mono we:text-text-primary we:tabular-nums">{value}</div>
      <div className="we:text-[9px] we:uppercase we:tracking-wide we:text-text-tertiary">
        {label}
      </div>
    </div>
  );
}

function Notice({ text }: { text: string }) {
  return (
    <div className="we:p-4">
      <div className="we:text-[11px] we:text-text-tertiary we:text-center we:py-6 we:border we:border-border-default we:rounded-lg">
        {text}
      </div>
    </div>
  );
}

export function AtlasRelayPage() {
  const host = useHost();
  const deviceId = host.node.deviceId;
  const [read, setRead] = useState<RelayReadState>({
    deviceId: null,
    last: null,
    snapshot: null,
    receivedAt: 0,
  });
  const [now, setNow] = useState(() => Date.now());
  const agentRef = useRef(host.agent);
  useEffect(() => {
    agentRef.current = host.agent;
  });

  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | undefined;

    const tick = async () => {
      if (cancelled) return;
      if (!document.hidden) {
        const next = await getAtlasRelayStatus(agentRef.current);
        if (cancelled) return;
        const at = Date.now();
        setRead((prev) => {
          const carried = prev.deviceId === deviceId ? prev : { snapshot: null, receivedAt: 0 };
          if (next.kind === "snapshot") {
            return { deviceId, last: next.kind, snapshot: next.status, receivedAt: at };
          }
          if (next.kind === "absent") {
            return { deviceId, last: next.kind, snapshot: null, receivedAt: 0 };
          }
          return {
            deviceId,
            last: next.kind,
            snapshot: carried.snapshot,
            receivedAt: carried.receivedAt,
          };
        });
        setNow(at);
      }
      if (!cancelled) timer = setTimeout(() => void tick(), POLL_INTERVAL_MS);
    };

    void tick();
    return () => {
      cancelled = true;
      clearTimeout(timer);
    };
  }, [deviceId]);

  const current = read.deviceId === deviceId && read.last !== null ? read : null;
  const status = current?.snapshot ?? null;
  const unreachable = current?.last === "unreachable";

  // Surface a card when a relay is running, or when the relay loop has gone
  // quiet (stale): a dead relay must not read as "no relay active".
  if (!status || (!status.stale && status.up !== true)) {
    if (unreachable) return <Notice text={t("relayUnreachable")} />;
    if (!current) return <Notice text={t("relayChecking")} />;
    return <Notice text={t("relayNoActive")} />;
  }

  const seen = status.datagramsSeen;
  const forwarded = status.forwarded;
  const keepRate =
    seen !== null && forwarded !== null && seen > 0
      ? `${Math.round((forwarded / seen) * 100)}%`
      : NO_DATA_GLYPH;
  const isStale = status.stale || unreachable || now - read.receivedAt > STALE_MS;
  const fmt = (v: number | null) => (v === null ? NO_DATA_GLYPH : String(v));

  return (
    <div className="we:p-4">
      <div className="we:border we:border-border-default we:rounded-lg we:p-4 we:space-y-3">
        <div className="we:flex we:items-center we:justify-between">
          <div className="we:flex we:items-center we:gap-1.5">
            <Boxes className="we:w-3.5 we:h-3.5 we:text-text-tertiary" aria-hidden="true" />
            <span className="we:text-xs we:font-medium we:text-text-secondary">
              {t("atlasRelay")}
            </span>
          </div>
          {isStale && (
            <span className="we:text-[10px] we:font-medium we:px-1.5 we:py-0.5 we:rounded we:bg-status-warning/15 we:text-status-warning">
              {t("stale")}
            </span>
          )}
        </div>

        <p className="we:text-[10px] we:text-text-tertiary">
          {unreachable ? t("relayUnreachable") : t("relayForwarding")}
        </p>

        <div className={cn("we:grid we:grid-cols-4 we:gap-2", isStale && "we:opacity-50")}>
          <Stat label={t("relaySeen")} value={fmt(status.datagramsSeen)} />
          <Stat label={t("relayForwarded")} value={fmt(status.forwarded)} />
          <Stat label={t("relayDropped")} value={fmt(status.malformed)} />
          <Stat label={t("relayFailed")} value={fmt(status.forwardFailed)} />
        </div>

        <div className="we:flex we:items-center we:justify-between we:border-t we:border-border-default we:pt-2">
          <span className="we:text-[10px] we:text-text-secondary">{t("relayKeepRate")}</span>
          <span className="we:text-[10px] we:font-mono we:text-text-primary we:tabular-nums">
            {keepRate}
          </span>
        </div>

        {status.computeUrl && (
          <div className="we:flex we:items-center we:justify-between">
            <span className="we:text-[10px] we:text-text-tertiary">{t("relayCompute")}</span>
            <span
              className="we:text-[10px] we:font-mono we:text-text-secondary we:truncate we:max-w-[60%]"
              title={status.computeUrl}
            >
              {status.computeUrl}
            </span>
          </div>
        )}
      </div>
    </div>
  );
}
