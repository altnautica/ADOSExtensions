/**
 * Plugin entry point.
 *
 * Loaded by the GCS plugin host inside a sandboxed iframe. The SDK
 * wraps the postMessage RPC envelope; this module wires the panel to
 * gimbal telemetry and config changes.
 */

import { definePlugin } from "@altnautica/plugin-sdk";

import { mountPanel, type PanelHandle } from "./panel";
import {
  DEFAULT_LIMITS,
  type AxisLimits,
  type GimbalState,
} from "./types";

interface ConfigShape {
  limits?: Partial<AxisLimits>;
  vehicleSystemId?: number;
  vehicleComponentId?: number;
}

let panel: PanelHandle | null = null;
let rootEl: HTMLElement | null = null;

definePlugin({
  id: "com.altnautica.mavlink-gimbal-v2",
  version: "1.0.0",
  async mount(ctx) {
    rootEl = document.getElementById("gimbal-root");
    if (!rootEl) {
      rootEl = document.createElement("div");
      rootEl.id = "gimbal-root";
      document.body.appendChild(rootEl);
    }
    panel = mountPanel(ctx, rootEl, { limits: DEFAULT_LIMITS });

    await ctx.telemetry.subscribe<GimbalState>("gimbal", (sample) => {
      if (sample && typeof sample.pitchDeg === "number") {
        panel?.setState(sample);
      }
    });
    ctx.config.onChange<ConfigShape>((next) => {
      const merged: AxisLimits = { ...DEFAULT_LIMITS, ...(next.limits ?? {}) };
      if (rootEl) {
        panel?.destroy();
        panel = mountPanel(ctx, rootEl, {
          limits: merged,
          vehicleSystemId: next.vehicleSystemId,
          vehicleComponentId: next.vehicleComponentId,
        });
      }
    });
  },
  async unmount() {
    panel?.destroy();
    panel = null;
    rootEl = null;
  },
});

export { panel as __testPanel };
