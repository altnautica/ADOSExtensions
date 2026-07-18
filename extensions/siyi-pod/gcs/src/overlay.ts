/**
 * The pod video HUD (video.overlay slot).
 *
 * A canvas laid over the live video that draws the reticle, the pod's
 * republished tracker boxes, and the range/zoom readout. It consumes the host's
 * letterbox-correct `video.overlay.props` and the pod state store. The host owns
 * click-to-track (the shared cockpit target overlay), so this overlay is a
 * read-only HUD.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import { drawHud } from "./overlay-draw";
import type { PodStateStore } from "./pod-state";
import type { VideoOverlayProps } from "./types";

export interface OverlayHandle {
  destroy(): void;
  /** True once the host has pushed at least one overlay-props frame. */
  hasProps(): boolean;
}

export function mountOverlay(
  ctx: PluginContext,
  root: HTMLElement,
  store: PodStateStore,
): OverlayHandle {
  const canvas = document.createElement("canvas");
  canvas.className = "siyi-hud";
  canvas.style.position = "absolute";
  canvas.style.inset = "0";
  canvas.style.width = "100%";
  canvas.style.height = "100%";
  canvas.style.pointerEvents = "none";
  root.appendChild(canvas);

  let props: VideoOverlayProps | null = null;
  let received = false;

  const draw = (): void => {
    const rect = props?.renderedRect;
    if (rect) {
      const w = Math.max(1, Math.round(rect.x * 2 + rect.width) || root.clientWidth);
      const h = Math.max(1, Math.round(rect.y * 2 + rect.height) || root.clientHeight);
      if (canvas.width !== w) canvas.width = w;
      if (canvas.height !== h) canvas.height = h;
    }
    drawHud(canvas.getContext("2d"), props, store.get());
  };

  const unsubProps = ctx.events.subscribe<VideoOverlayProps>(
    "video.overlay.props",
    (p) => {
      received = true;
      props = p;
      draw();
    },
  );
  const unsubState = store.subscribe(() => draw());

  return {
    destroy() {
      unsubProps();
      unsubState();
      canvas.remove();
    },
    hasProps() {
      return received;
    },
  };
}
