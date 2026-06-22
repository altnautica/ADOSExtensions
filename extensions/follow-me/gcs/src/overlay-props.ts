/**
 * Video-overlay host-prop subscription.
 *
 * A `video.overlay` iframe receives a VideoOverlayHostProps payload pushed
 * by the cockpit on the non-gated `video.overlay.props` event: the
 * rendered (letterbox-corrected) rect, the stream resolution, the latest
 * attitude, and the latest detection batch. A `drone.detail.tab` mount of
 * the same bundle never receives this event, which is how the plugin tells
 * the two slots apart at runtime.
 *
 * @license GPL-3.0-or-later
 */

import { useEffect, useState } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import {
  VIDEO_OVERLAY_PROPS_EVENT,
  type VideoOverlayHostProps,
} from "./types";

/**
 * Subscribe to the overlay host props. Returns the latest payload, or null
 * before the first one arrives. The presence of any payload also signals
 * that this iframe is the video-overlay slot.
 */
export function useOverlayProps(
  ctx: PluginContext,
): VideoOverlayHostProps | null {
  const [props, setProps] = useState<VideoOverlayHostProps | null>(null);

  useEffect(() => {
    const off = ctx.events.subscribe<VideoOverlayHostProps>(
      VIDEO_OVERLAY_PROPS_EVENT,
      (next) => {
        if (next && typeof next === "object") setProps(next);
      },
    );
    return off;
  }, [ctx]);

  return props;
}
