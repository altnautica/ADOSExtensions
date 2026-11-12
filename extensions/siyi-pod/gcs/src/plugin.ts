/**
 * SIYI Optical Pod plugin entry point.
 *
 * Built into `plugin.bundle.js` and loaded by the GCS plugin host inside a
 * sandboxed iframe. One bundle serves the pod's contribution slots: it mounts
 * the control console and a read-only video HUD, subscribes to the pod state,
 * and — when the host pushes video-overlay props (the video.overlay slot) —
 * hides the console so only the HUD paints over the live video. The host loads
 * only the bundle, so the stylesheet is bundled as text and injected at mount.
 *
 * @license GPL-3.0-or-later
 */

import { definePlugin, type PluginContext } from "@altnautica/plugin-sdk";

import css from "../plugin.css";
import { mountSiyi } from "./mount";
import { injectStyles } from "./styles";
import { PLUGIN_VERSION } from "./version";
import enLocale from "../../locales/en.json";

const PLUGIN_ID = "com.altnautica.siyi-pod";

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
    injectStyles(css);
    handles = await mountSiyi(ctx, ensureRoot());
  },
  async unmount() {
    for (const h of handles) h.destroy();
    handles = [];
    if (rootEl?.parentNode) rootEl.parentNode.removeChild(rootEl);
    rootEl = null;
  },
});
