/**
 * Plugin entry point. Built into `plugin.bundle.js` and loaded by the
 * GCS plugin host inside a sandboxed iframe. The Altnautica plugin
 * SDK wraps the postMessage RPC envelope; this module just wires the
 * battery store to telemetry, notifications, and recording markers.
 */

import { definePlugin } from "@altnautica/plugin-sdk";

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

const store = createBatteryStore(DEFAULT_CONFIG);
let rootEl: HTMLElement | null = null;

definePlugin({
  id: "com.altnautica.battery-health-panel",
  version: "1.0.0",
  async mount(ctx) {
    rootEl = document.getElementById("battery-health-root");
    if (!rootEl) {
      rootEl = document.createElement("div");
      rootEl.id = "battery-health-root";
      document.body.appendChild(rootEl);
    }
    render();

    store.subscribe(render);
    store.onAnomaly(async (fired) => {
      for (const event of fired) {
        try {
          await ctx.notifications.publish({
            channelId: "battery-anomaly",
            severity: event.severity,
            title: event.title,
            body: event.body,
            meta: { packId: event.packId, ruleId: event.ruleId },
          });
          await ctx.recording.mark({
            label: event.title,
            meta: { packId: event.packId, ruleId: event.ruleId },
          });
        } catch {
          // Host denied or no recording active. Anomaly stays in the
          // panel regardless.
        }
      }
    });

    await ctx.telemetry.subscribe<BatterySample>("battery", (sample) => {
      if (sample && Array.isArray(sample.cellVoltagesV)) {
        store.ingest(sample);
      }
    });
    await ctx.telemetry.subscribe<StatustextMessage>(
      "mavlink.STATUSTEXT",
      (msg) => {
        if (msg && typeof msg.text === "string") {
          store.ingestStatustext(msg);
        }
      },
    );
    ctx.config.onChange<BatteryHealthConfig>((next) => {
      store.setConfig(next);
    });
  },
});

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
      card.appendChild(kv("Total", formatVolts(sample.totalVoltageV)));
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

function kv(label: string, value: string): HTMLElement {
  const row = document.createElement("div");
  row.className = "bhp-row";
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

export { store as __testStore };
