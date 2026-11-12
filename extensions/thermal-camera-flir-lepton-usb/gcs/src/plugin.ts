/**
 * Plugin entry point. Built into ``plugin.bundle.js`` and loaded by
 * the GCS plugin host inside a sandboxed iframe. The plugin renders
 * a thermal video overlay on top of the host's video pane; see
 * {@link mountOverlay}.
 */

import { definePlugin } from "@altnautica/plugin-sdk";

import enLocale from "../../locales/en.json";
import { mountOverlay, type OverlayHandle } from "./overlay";

export const PLUGIN_VERSION = "1.4.2";

let handle: OverlayHandle | null = null;

definePlugin({
  id: "com.altnautica.thermal-flir-lepton-usb",
  version: PLUGIN_VERSION,
  locale: enLocale as Record<string, string>,
  async mount(ctx) {
    handle = await mountOverlay(ctx, document);
  },
  unmount() {
    handle?.unmount();
    handle = null;
  },
});
