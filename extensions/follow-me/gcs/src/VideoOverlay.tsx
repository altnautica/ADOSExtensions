/**
 * Follow-Me interactive video overlay.
 *
 * Draws the live detection boxes over the rendered video rect and lets the
 * operator click one to designate the follow subject. The overlay is
 * dormant (pointer-events off, so the video pane stays interactive) until
 * the Follow-Me skill is active; while active and not yet locked it shows a
 * designation reticle and the boxes become clickable. A click maps back to
 * a source-frame pixel, hit-tests the smallest containing box, and forwards
 * the designation to the agent, which owns the resulting lock.
 *
 * @license GPL-3.0-or-later
 */

import { useCallback, useRef, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { buildDesignatePayload, sendDesignate } from "./designate";
import { clientToFramePixel, hitTestDetection, placeBox } from "./overlay-geometry";
import { lockColor } from "./style";
import type {
  FollowState,
  OverlayDetectionItem,
  VideoOverlayHostProps,
} from "./types";

export interface VideoOverlayProps {
  ctx: PluginContext;
  hostProps: VideoOverlayHostProps;
  follow: FollowState;
  /** Test seam: observe a designation dispatch result. */
  onDesignateResult?: (ok: boolean, det: OverlayDetectionItem) => void;
}

export function VideoOverlay({
  ctx,
  hostProps,
  follow,
  onDesignateResult,
}: VideoOverlayProps): JSX.Element {
  const wrapperRef = useRef<HTMLDivElement | null>(null);

  // The overlay accepts clicks only while the skill is active and the agent
  // has not yet locked a subject. Once locked, clicks pass through so the
  // operator can re-take control of the video pane; re-designating requires
  // toggling back to the designate phase (lock lost or skill re-armed).
  const designating = follow.active && follow.lockState !== "locked";

  const detections = hostProps.detections?.items ?? [];
  const frameWidth = hostProps.detections?.frameWidth ?? hostProps.streamWidth;
  const frameHeight =
    hostProps.detections?.frameHeight ?? hostProps.streamHeight;

  const handleClick = useCallback(
    (ev: React.MouseEvent<HTMLDivElement>) => {
      if (!designating) return;
      const wrapper = wrapperRef.current;
      if (!wrapper) return;
      const wrapperRect = wrapper.getBoundingClientRect();
      const clientX = ev.clientX - wrapperRect.left;
      const clientY = ev.clientY - wrapperRect.top;
      const framePoint = clientToFramePixel(
        clientX,
        clientY,
        frameWidth,
        frameHeight,
        hostProps.renderedRect,
      );
      if (!framePoint) return;
      const hit = hitTestDetection(framePoint, detections);
      if (!hit) return;
      const payload = buildDesignatePayload(hostProps.cameraId, hit);
      void sendDesignate(ctx, payload).then((ok) => {
        onDesignateResult?.(ok, hit);
      });
    },
    [
      ctx,
      designating,
      detections,
      frameWidth,
      frameHeight,
      hostProps.cameraId,
      hostProps.renderedRect,
      onDesignateResult,
    ],
  );

  const wrapperStyle: CSSProperties = {
    position: "absolute",
    inset: 0,
    // Dormant overlay must not steal pointer events from the video pane.
    pointerEvents: designating ? "auto" : "none",
    cursor: designating ? "crosshair" : "default",
  };

  return (
    <div
      ref={wrapperRef}
      style={wrapperStyle}
      onClick={handleClick}
      data-testid="fm-video-overlay"
      data-designating={designating ? "true" : "false"}
    >
      {designating ? <Reticle ctx={ctx} /> : null}
      {detections.map((det, i) => {
        const placed = placeBox(
          det.bbox,
          frameWidth,
          frameHeight,
          hostProps.renderedRect,
        );
        const locked = det.trackId != null && det.trackId === follow.targetId;
        const color = locked
          ? lockColor(follow.lockState)
          : lockColor(det.trackId != null ? det.lockState : null);
        const boxStyle: CSSProperties = {
          position: "absolute",
          left: `${placed.left}px`,
          top: `${placed.top}px`,
          width: `${placed.width}px`,
          height: `${placed.height}px`,
          border: `${locked ? 2 : 1}px solid ${color}`,
          boxSizing: "border-box",
          pointerEvents: designating ? "auto" : "none",
          cursor: designating ? "pointer" : "default",
        };
        const pct = Math.round(det.confidence * 100);
        const label =
          det.trackId != null
            ? `${det.classLabel} #${det.trackId} ${pct}%`
            : `${det.classLabel} ${pct}%`;
        return (
          <div
            key={`${hostProps.detections?.frameId ?? 0}-${i}`}
            style={boxStyle}
            role={designating ? "button" : undefined}
            title={
              designating ? ctx.i18n.t("overlay.clickToFollow") : undefined
            }
            data-testid="fm-detection-box"
            data-track-id={det.trackId ?? ""}
            data-locked={locked ? "true" : "false"}
          >
            <span
              style={{
                position: "absolute",
                left: 0,
                top: 0,
                transform: "translateY(-100%)",
                whiteSpace: "nowrap",
                background: "rgba(0,0,0,0.7)",
                color,
                padding: "0 0.25rem",
                fontFamily: "monospace",
                fontSize: "10px",
                lineHeight: 1.2,
              }}
            >
              {label}
            </span>
          </div>
        );
      })}
    </div>
  );
}

/** A faint center reticle that signals the overlay is in designate mode. */
function Reticle({ ctx }: { ctx: PluginContext }): JSX.Element {
  const hint = ctx.i18n.t("overlay.designateHint");
  return (
    <div
      style={{ position: "absolute", inset: 0, pointerEvents: "none" }}
      aria-hidden
      data-testid="fm-reticle"
    >
      <div
        style={{
          position: "absolute",
          left: "50%",
          top: "50%",
          width: "2rem",
          height: "2rem",
          transform: "translate(-50%, -50%)",
          border: "1px solid var(--fm-warning, #f59e0b)",
          borderRadius: "999px",
        }}
      />
      <div
        style={{
          position: "absolute",
          left: "50%",
          bottom: "0.5rem",
          transform: "translateX(-50%)",
          background: "rgba(0,0,0,0.7)",
          color: "var(--fm-text, #e5e7eb)",
          padding: "0.2rem 0.5rem",
          borderRadius: "0.25rem",
          fontSize: "0.7rem",
          whiteSpace: "nowrap",
        }}
      >
        {hint}
      </div>
    </div>
  );
}
