/**
 * Mounts the console and the video HUD into one root and wires the pod feed.
 *
 * Overlay props arrive only in the video.overlay slot; when they do the console
 * is hidden inline, so only the HUD paints over the video whether or not the
 * stylesheet has loaded.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import { mountOverlay } from "./overlay";
import { mountPanel } from "./panel";
import { PodStateStore } from "./pod-state";

export async function mountSiyi(
  ctx: PluginContext,
  root: HTMLElement,
): Promise<{ destroy(): void }[]> {
  const store = new PodStateStore();
  const panel = mountPanel(ctx, root, store);
  const overlay = mountOverlay(ctx, root, store);
  const unsubProps = ctx.videoOverlay.onProps(() => {
    root.classList.add("siyi-overlay-mode");
    panel.hide();
  });

  // Live pod read-back rides the "siyi" telemetry channel and the
  // siyi.pod.state event; either updates the store, each aged on its own
  // clock.
  const unsubTelemetry = await ctx.telemetry.subscribe("siyi", (s) =>
    store.ingest(s, "telemetry"),
  );
  const unsubEvents = ctx.events.subscribe("siyi.pod.state", (s) => store.ingest(s, "event"));
  return [
    panel,
    overlay,
    {
      destroy() {
        unsubProps();
        unsubTelemetry();
        unsubEvents();
      },
    },
  ];
}
