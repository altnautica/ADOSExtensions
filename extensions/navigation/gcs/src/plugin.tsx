/**
 * Plugin entry point. Built into ``plugin.bundle.js`` and loaded by
 * the GCS plugin host inside a sandboxed iframe. One bundle serves two
 * contributions and routes on the one it was mounted as
 * (`ctx.mount.panelId`): the cockpit panel `navigation-health` and the
 * Agent page `navigation`.
 */

import { StrictMode } from "react";
import type { Root } from "react-dom/client";
import { createRoot } from "react-dom/client";

import { definePlugin, type PluginContext } from "@altnautica/plugin-sdk";

import { Page } from "./components/Page";
import { firmwareFromConfig, type NavigationConfig } from "./firmware";
import { FLAT_LOCALE } from "./i18n";
import type { FirmwareType } from "./types";

const DEFAULT_FIRMWARE: FirmwareType = "ardupilot";

let rootEl: HTMLElement | null = null;
let reactRoot: Root | null = null;
let firmware: FirmwareType = DEFAULT_FIRMWARE;

function ensureRootEl(): HTMLElement {
  if (rootEl) return rootEl;
  let el = document.getElementById("navigation-root");
  if (!el) {
    el = document.createElement("div");
    el.id = "navigation-root";
    document.body.appendChild(el);
  }
  rootEl = el;
  return el;
}

function renderTree(ctx: PluginContext): void {
  if (!reactRoot) return;
  reactRoot.render(
    <StrictMode>
      <Page ctx={ctx} firmware={firmware} />
    </StrictMode>,
  );
}

definePlugin({
  id: "com.altnautica.navigation",
  version: "1.0.0",
  locale: FLAT_LOCALE,
  async mount(ctx) {
    const host = ensureRootEl();
    reactRoot = createRoot(host);
    renderTree(ctx);

    ctx.config.onChange<NavigationConfig>((next) => {
      const nextFirmware = firmwareFromConfig(next);
      if (nextFirmware) {
        firmware = nextFirmware;
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
// useEffect cleanup in `useNavigationTelemetry` handles its own
// subscription teardown when the iframe is replayed by the host.
export const __test = {
  getFirmware: (): FirmwareType => firmware,
  setFirmware: (f: FirmwareType): void => {
    firmware = f;
  },
};
