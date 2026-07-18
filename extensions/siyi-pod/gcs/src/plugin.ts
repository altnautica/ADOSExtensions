/**
 * SIYI Optical Pod plugin entry point.
 *
 * Built into `plugin.bundle.js` and loaded by the GCS plugin host inside a
 * sandboxed iframe. One bundle serves the pod's contribution slots: it mounts
 * the control console and a read-only video HUD, subscribes to the pod state,
 * and — when the host pushes video-overlay props (the video.overlay slot) —
 * hides the console so only the HUD paints over the live video.
 *
 * @license GPL-3.0-or-later
 */

import { definePlugin, type PluginContext } from "@altnautica/plugin-sdk";

import { mountOverlay } from "./overlay";
import { mountPanel } from "./panel";
import { PodStateStore } from "./pod-state";
import type { PodState } from "./types";
import enLocale from "../../locales/en.json";

const PLUGIN_ID = "com.altnautica.siyi-pod";
const PLUGIN_VERSION = "0.3.0";

let handles: { destroy(): void }[] = [];
let rootEl: HTMLElement | null = null;

function ensureRoot(): HTMLElement {
  if (rootEl) return rootEl;
  let el = document.getElementById("siyi-pod-root");
  if (!el) {
    el = document.createElement("div");
    el.id = "siyi-pod-root";
    el.style.position = "relative";
    el.style.height = "100%";
    document.body.appendChild(el);
  }
  rootEl = el;
  return el;
}

definePlugin({
  id: PLUGIN_ID,
  version: PLUGIN_VERSION,
  locale: enLocale as Record<string, string>,
  async mount(ctx: PluginContext) {
    const root = ensureRoot();
    const store = new PodStateStore();
    handles = [mountPanel(ctx, root, store), mountOverlay(ctx, root, store)];

    // Overlay props only arrive in the video.overlay slot; when they do, hide
    // the console so the HUD paints alone over the video.
    ctx.events.subscribe<unknown>("video.overlay.props", () => {
      root.classList.add("siyi-overlay-mode");
    });

    // Live pod read-back rides the "siyi" telemetry channel and the
    // siyi.pod.state event; either updates the store.
    await ctx.telemetry.subscribe<PodState>("siyi", (s) => store.ingest(s));
    ctx.events.subscribe<PodState>("siyi.pod.state", (s) => store.ingest(s));
  },
  async unmount() {
    for (const h of handles) h.destroy();
    handles = [];
    if (rootEl?.parentNode) rootEl.parentNode.removeChild(rootEl);
    rootEl = null;
  },
});
