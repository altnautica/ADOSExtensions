/**
 * Follow-Me plugin entry point. Built into ``plugin.bundle.js`` and loaded
 * by the GCS plugin host inside a sandboxed iframe, once per slot context.
 *
 * The same bundle backs three slot contributions. The skill is declarative
 * (the host builds it from the manifest) so this entry only renders the two
 * iframe surfaces: the interactive video overlay and the drone-detail tab.
 * The two are told apart at runtime by the host-prop stream: a
 * ``video.overlay`` mount receives the ``video.overlay.props`` event; a
 * ``drone.detail.tab`` mount never does. The root renders the tab until an
 * overlay-props payload arrives, at which point it switches to the overlay
 * (the overlay iframe receives that payload on load, so the swap is
 * immediate and invisible behind the video pane it is sized over).
 *
 * @license GPL-3.0-or-later
 */

import { StrictMode } from "react";
import type { Root } from "react-dom/client";
import { createRoot } from "react-dom/client";

import { definePlugin, type PluginContext } from "@altnautica/plugin-sdk";

import { FollowMeTab } from "./FollowMeTab";
import { useFollowState } from "./follow-state";
import { useOverlayProps } from "./overlay-props";
import { VideoOverlay } from "./VideoOverlay";
import enLocale from "../../locales/en.json";

const PLUGIN_ID = "com.altnautica.follow-me";
const PLUGIN_VERSION = "0.1.0";

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

/**
 * The single iframe surface. It owns the overlay-props + follow-state
 * subscriptions and renders the overlay when overlay props are present, the
 * tab otherwise. Both are mounted from the same bundle in separate iframes;
 * the host-prop stream is what distinguishes the slot.
 */
function FollowMeRoot({ ctx }: { ctx: PluginContext }): JSX.Element {
  const overlayProps = useOverlayProps(ctx);
  const follow = useFollowState(ctx);

  if (overlayProps) {
    return <VideoOverlay ctx={ctx} hostProps={overlayProps} follow={follow} />;
  }
  return <FollowMeTab ctx={ctx} followOverride={follow} />;
}

function renderTree(ctx: PluginContext): void {
  if (!reactRoot) return;
  reactRoot.render(
    <StrictMode>
      <FollowMeRoot ctx={ctx} />
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
