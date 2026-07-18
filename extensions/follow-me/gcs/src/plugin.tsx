/**
 * Follow-Me plugin entry point. Built into ``plugin.bundle.js`` and loaded
 * by the GCS plugin host inside a sandboxed iframe.
 *
 * The GCS half now renders a single iframe surface: the node-detail tab
 * (Specs + live read-back). The skill is declarative (the host builds it
 * from the manifest), the plain settings render as native parameters, and
 * the click-to-follow interaction is a host-owned cockpit target action —
 * the host draws the detection boxes, owns the selection, and runs the
 * designate + config-write. So this plugin no longer ships its own video
 * overlay.
 *
 * @license GPL-3.0-or-later
 */

import { StrictMode } from "react";
import type { Root } from "react-dom/client";
import { createRoot } from "react-dom/client";

import { definePlugin, type PluginContext } from "@altnautica/plugin-sdk";

import { FollowMeTab } from "./FollowMeTab";
import enLocale from "../../locales/en.json";

const PLUGIN_ID = "com.altnautica.follow-me";
const PLUGIN_VERSION = "0.2.2";

let rootEl: HTMLElement | null = null;
let reactRoot: Root | null = null;

function ensureRootEl(): HTMLElement {
  if (rootEl) return rootEl;
  let el = document.getElementById("follow-me-root");
  if (!el) {
    el = document.createElement("div");
    el.id = "follow-me-root";
    el.style.height = "100%";
    document.body.appendChild(el);
  }
  rootEl = el;
  return el;
}

function renderTree(ctx: PluginContext): void {
  if (!reactRoot) return;
  reactRoot.render(
    <StrictMode>
      <FollowMeTab ctx={ctx} />
    </StrictMode>,
  );
}

definePlugin({
  id: PLUGIN_ID,
  version: PLUGIN_VERSION,
  locale: enLocale as Record<string, string>,
  mount(ctx) {
    const host = ensureRootEl();
    reactRoot = createRoot(host);
    renderTree(ctx);
  },
  unmount() {
    if (reactRoot) {
      reactRoot.unmount();
      reactRoot = null;
    }
    if (rootEl && rootEl.parentNode) {
      rootEl.parentNode.removeChild(rootEl);
    }
    rootEl = null;
  },
});
