/**
 * Plugin entry point. Loaded by the GCS plugin host inside a sandboxed
 * iframe (sandbox=allow-scripts, null origin). All host I/O round-trips
 * through `postMessage` envelopes; we never reach into `window.top`.
 *
 * The entry registers with the host, subscribes to telemetry, runs the
 * store + rule engine, emits anomaly notifications, and renders a
 * minimal panel. The panel UI is intentionally light at v1.0; richer
 * components arrive in a follow-up release.
 */

import { createBatteryStore } from "./batteryStore";
import {
  formatAmps,
  formatCelsius,
  formatEta,
  formatPercent,
  formatVolts,
} from "./formatters";
import {
  DEFAULT_CONFIG,
  type BatteryHealthConfig,
  type BatterySample,
  type StatustextMessage,
} from "./types";

interface RpcEnvelope {
  id: string;
  type: "request" | "response" | "event";
  method: string;
  capability: string;
  args: unknown;
  version: 1;
  error?: { code: string; message: string };
}

const PROTOCOL_VERSION = 1;

const store = createBatteryStore(DEFAULT_CONFIG);
let rootEl: HTMLElement | null = null;

function callHost<T = unknown>(
  method: string,
  capability: string,
  args: unknown,
): Promise<T> {
  return new Promise((resolve, reject) => {
    const id = crypto.randomUUID();
    const onMessage = (ev: MessageEvent<RpcEnvelope>): void => {
      const data = ev.data;
      if (!data || data.id !== id || data.type !== "response") return;
      window.removeEventListener("message", onMessage);
      if (data.error) {
        reject(new Error(`${data.error.code}: ${data.error.message}`));
        return;
      }
      resolve(data.args as T);
    };
    window.addEventListener("message", onMessage);
    const env: RpcEnvelope = {
      id,
      type: "request",
      method,
      capability,
      args,
      version: PROTOCOL_VERSION,
    };
    window.parent.postMessage(env, "*");
  });
}

function listenHostEvents(handler: (env: RpcEnvelope) => void): () => void {
  const onMessage = (ev: MessageEvent<RpcEnvelope>): void => {
    const data = ev.data;
    if (!data || data.type !== "event") return;
    handler(data);
  };
  window.addEventListener("message", onMessage);
  return () => window.removeEventListener("message", onMessage);
}

async function bootstrap(): Promise<void> {
  rootEl = document.getElementById("battery-health-root");
  if (!rootEl) {
    rootEl = document.createElement("div");
    rootEl.id = "battery-health-root";
    document.body.appendChild(rootEl);
  }
  render();

  store.onAnomaly(async (fired) => {
    for (const event of fired) {
      try {
        await callHost("notification.publish", "ui.slot.notification", {
          channelId: "battery-anomaly",
          severity: event.severity,
          title: event.title,
          body: event.body,
          meta: { packId: event.packId, ruleId: event.ruleId },
        });
        await callHost("recording.mark", "recording.write", {
          label: event.title,
          meta: { packId: event.packId, ruleId: event.ruleId },
        });
      } catch {
        // Host denied or no recording active. Anomaly stays in the
        // panel regardless.
      }
    }
  });

  store.subscribe(render);

  await callHost(
    "telemetry.subscribe",
    "telemetry.subscribe.battery",
    { topic: "battery" },
  );
  await callHost(
    "telemetry.subscribe",
    "telemetry.subscribe.mavlink",
    { topic: "mavlink.STATUSTEXT" },
  );

  listenHostEvents((env) => {
    if (env.method === "telemetry.battery") {
      const sample = env.args as BatterySample;
      if (sample && Array.isArray(sample.cellVoltagesV)) {
        store.ingest(sample);
      }
    } else if (env.method === "telemetry.mavlink.STATUSTEXT") {
      const msg = env.args as StatustextMessage;
      if (msg && typeof msg.text === "string") {
        store.ingestStatustext(msg);
      }
    } else if (env.method === "config.changed") {
      const next = env.args as BatteryHealthConfig | undefined;
      if (next) store.setConfig(next);
    }
  });
}

function render(): void {
  if (!rootEl) return;
  const snap = store.getSnapshot();
  rootEl.innerHTML = "";
  if (snap.packs.length === 0) {
    rootEl.appendChild(text("p", "Awaiting battery telemetry..."));
    return;
  }
  for (const pack of snap.packs) {
    const card = document.createElement("section");
    card.className = "bhp-pack";
    card.setAttribute("data-pack-id", String(pack.packId));
    card.appendChild(text("h2", `Pack ${pack.packId}`));

    const sample = pack.latestSample;
    if (sample) {
      card.appendChild(
        kv("Total", formatVolts(sample.totalVoltageV), "bhp-row"),
      );
      card.appendChild(kv("Current", formatAmps(sample.currentA)));
      card.appendChild(kv("Remaining", formatPercent(sample.remainingPercent)));
      card.appendChild(kv("Temp", formatCelsius(sample.temperatureC)));
      const cells = document.createElement("ul");
      cells.className = "bhp-cells";
      sample.cellVoltagesV.forEach((v, i) => {
        const li = document.createElement("li");
        li.textContent = `Cell ${i + 1}: ${formatVolts(v)}`;
        cells.appendChild(li);
      });
      card.appendChild(cells);
    }

    if (pack.predictive) {
      card.appendChild(
        kv("Time to reserve", formatEta(pack.predictive.etaSec)),
      );
    }

    if (pack.liveAnomalies.size > 0) {
      const list = document.createElement("ul");
      list.className = "bhp-anomalies";
      for (const event of pack.liveAnomalies.values()) {
        const li = document.createElement("li");
        li.className = `bhp-anomaly bhp-anomaly--${event.severity}`;
        li.textContent = `${event.title}: ${event.body}`;
        list.appendChild(li);
      }
      card.appendChild(list);
    }
    rootEl.appendChild(card);
  }
}

function text(tag: keyof HTMLElementTagNameMap, body: string): HTMLElement {
  const el = document.createElement(tag);
  el.textContent = body;
  return el;
}

function kv(label: string, value: string, cls = "bhp-row"): HTMLElement {
  const row = document.createElement("div");
  row.className = cls;
  const k = document.createElement("span");
  k.className = "bhp-key";
  k.textContent = label;
  const v = document.createElement("span");
  v.className = "bhp-value";
  v.textContent = value;
  row.appendChild(k);
  row.appendChild(v);
  return row;
}

if (typeof window !== "undefined") {
  if (document.readyState === "complete" || document.readyState === "interactive") {
    void bootstrap();
  } else {
    window.addEventListener("DOMContentLoaded", () => void bootstrap());
  }
}

export { store as __testStore };
