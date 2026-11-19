/**
 * @module pages/ground-station/RelayPage
 * @description Ground-station capture relay indicator. Polls the node's relay
 * status through this extension's agent server and, only when a relay is
 * actually running (`up === true`) or has gone quiet (stale), shows the
 * keyframes-seen / forwarded / keep-rate counters plus a staleness badge. A
 * failed read says the node is unreachable (keeping the last snapshot,
 * badged stale), never "no relay". One request is in flight at a time, the
 * next is armed once it settles, and a hidden document skips the read.
 */

import { useEffect, useRef, useState } from "react";
import { Boxes } from "lucide-react";

import { useHost } from "@altnautica/mapping-ui/host/context";
import { translator } from "../host/i18n";
import {
  getRelayStatus,
  type CaptureRelayRead,
  type CaptureRelayStatus,
} from "../lib/agent/relay-client";
import { cn, NO_DATA_GLYPH } from "@altnautica/mapping-ui/lib/utils";

const t = translator("setup");

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
  last: CaptureRelayRead["kind"] | null;
  snapshot: CaptureRelayStatus | null;
  receivedAt: number;
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="cap:rounded cap:bg-bg-tertiary cap:px-2 cap:py-1.5 cap:text-center">
      <div className="cap:text-sm cap:font-mono cap:text-text-primary cap:tabular-nums">{value}</div>
      <div className="cap:text-[9px] cap:uppercase cap:tracking-wide cap:text-text-tertiary">
        {label}
      </div>
    </div>
  );
}

function Notice({ text }: { text: string }) {
  return (
    <div className="cap:p-4">
      <div className="cap:text-[11px] cap:text-text-tertiary cap:text-center cap:py-6 cap:border cap:border-border-default cap:rounded-lg">
        {text}
      </div>
    </div>
  );
}

export function RelayPage() {
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
        const next = await getRelayStatus(agentRef.current);
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
    <div className="cap:p-4">
      <div className="cap:border cap:border-border-default cap:rounded-lg cap:p-4 cap:space-y-3">
        <div className="cap:flex cap:items-center cap:justify-between">
          <div className="cap:flex cap:items-center cap:gap-1.5">
            <Boxes className="cap:w-3.5 cap:h-3.5 cap:text-text-tertiary" aria-hidden="true" />
            <span className="cap:text-xs cap:font-medium cap:text-text-secondary">
              {t("relayTitle")}
            </span>
          </div>
          {isStale && (
            <span className="cap:text-[10px] cap:font-medium cap:px-1.5 cap:py-0.5 cap:rounded cap:bg-status-warning/15 cap:text-status-warning">
              {t("stale")}
            </span>
          )}
        </div>

        <p className="cap:text-[10px] cap:text-text-tertiary">
          {unreachable ? t("relayUnreachable") : t("relayForwarding")}
        </p>

        <div className={cn("cap:grid cap:grid-cols-4 cap:gap-2", isStale && "cap:opacity-50")}>
          <Stat label={t("relaySeen")} value={fmt(status.datagramsSeen)} />
          <Stat label={t("relayForwarded")} value={fmt(status.forwarded)} />
          <Stat label={t("relayDropped")} value={fmt(status.malformed)} />
          <Stat label={t("relayFailed")} value={fmt(status.forwardFailed)} />
        </div>

        <div className="cap:flex cap:items-center cap:justify-between cap:border-t cap:border-border-default cap:pt-2">
          <span className="cap:text-[10px] cap:text-text-secondary">{t("relayKeepRate")}</span>
          <span className="cap:text-[10px] cap:font-mono cap:text-text-primary cap:tabular-nums">
            {keepRate}
          </span>
        </div>

        {status.computeUrl && (
          <div className="cap:flex cap:items-center cap:justify-between">
            <span className="cap:text-[10px] cap:text-text-tertiary">{t("relayCompute")}</span>
            <span
              className="cap:text-[10px] cap:font-mono cap:text-text-secondary cap:truncate cap:max-w-[60%]"
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
