/**
 * Plugin entry point. Built into ``plugin.bundle.js`` and loaded by
 * the GCS plugin host inside a sandboxed iframe. The SDK wraps the
 * postMessage RPC envelope; this module mounts the Navigation tab
 * React tree into a host-provided container, subscribes to navigation
 * telemetry, and replays the wiring on per-drone lifecycle events.
 */

import { StrictMode } from "react";
import type { Root } from "react-dom/client";
import { createRoot } from "react-dom/client";

import { definePlugin } from "@altnautica/plugin-sdk";

import en from "../../locales/en.json";

import { NavigationTab } from "./components/NavigationTab";
import type { FirmwareType } from "./types";

interface VisionNavConfig {
  firmware?: FirmwareType;
}

const DEFAULT_FIRMWARE: FirmwareType = "ardupilot";

type LocaleTree = { [key: string]: string | LocaleTree };

/**
 * Flatten the shipped locale tree into the dotted keys the host's i18n
 * lookup uses. `locales/en.json` is the single source for this copy —
 * the manifest ships it and the bundle embeds the same file, so the two
 * cannot drift.
 */
function flattenLocale(tree: LocaleTree, prefix: string): Record<string, string> {
  const out: Record<string, string> = {};
  for (const [key, value] of Object.entries(tree)) {
    const path = `${prefix}${key}`;
    if (typeof value === "string") {
      out[path] = value;
    } else {
      Object.assign(out, flattenLocale(value, `${path}.`));
    }
  }
  return out;
}

const FLAT_LOCALE = flattenLocale(en as LocaleTree, "navigation.");

let rootEl: HTMLElement | null = null;
let reactRoot: Root | null = null;
let firmware: FirmwareType = DEFAULT_FIRMWARE;

function ensureRootEl(): HTMLElement {
  if (rootEl) return rootEl;
  let el = document.getElementById("vision-nav-root");
  if (!el) {
    el = document.createElement("div");
    el.id = "vision-nav-root";
    document.body.appendChild(el);
  }
  rootEl = el;
  return el;
}

function renderTree(ctx: import("@altnautica/plugin-sdk").PluginContext): void {
  if (!reactRoot) return;
  reactRoot.render(
    <StrictMode>
      <NavigationTab ctx={ctx} firmware={firmware} />
    </StrictMode>,
  );
}

definePlugin({
  id: "com.altnautica.vision-nav",
  version: "0.4.1",
  locale: FLAT_LOCALE,
  async mount(ctx) {
    const host = ensureRootEl();
    reactRoot = createRoot(host);
    renderTree(ctx);

    ctx.config.onChange<VisionNavConfig>((next) => {
      if (next && typeof next.firmware === "string") {
        firmware = next.firmware;
        renderTree(ctx);
      }
    });
  },
  async unmount() {
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

// Per-drone lifecycle hooks are not yet part of the SDK's
// `definePlugin` surface (mount/unmount only). The agent re-publishes
// the navigation telemetry topic on drone-switch, and the React
// useEffect cleanup in `useVisionNavTelemetry` handles its own
// subscription teardown when the iframe is replayed by the host.
export const __test = {
  getFirmware: (): FirmwareType => firmware,
  setFirmware: (f: FirmwareType): void => {
    firmware = f;
  },
};
