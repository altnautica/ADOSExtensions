/**
 * The thermal video overlay: the spot marker, the temperature readout and
 * the agent-applied palette over the host's video pane.
 *
 * It subscribes to two host channels: `thermal` for whether the agent holds
 * an open radiometric session (and which palette it applied), and
 * `camera.thermal.frame` for the frame read-backs that only exist while one
 * is open. The readout renders from both, so an absent sensor reads as
 * unavailable rather than as a camera whose frames are still on their way,
 * and a reading that stopped refreshing reads as stopped rather than live.
 *
 * The palette is changed through the `palette` parameter and the
 * cycle-palette Skill, both of which reach the agent; the overlay only shows
 * the palette the agent reports as applied.
 */

import { SampleClock, type PluginContext } from "@altnautica/plugin-sdk";

import { paintFrame } from "./render";
import { celsiusAt, clientToFrame, frameToCanvas, readSpot } from "./spotMeter";
import {
  paletteText,
  parseThermalState,
  readout,
  type Readout,
  type ThermalState,
} from "./status";
import { injectStyle } from "./style";
import {
  DEFAULT_THERMAL_CONFIG,
  type SpotMeterState,
  type ThermalCameraConfig,
  type ThermalFrame,
} from "./types";

/** Re-render cadence, so the reading's age keeps counting between frames. */
const TICK_MS = 1000;

export interface OverlayHandle {
  unmount(): void;
  getReadout(): Readout;
}

export async function mountOverlay(
  ctx: PluginContext,
  doc: Document,
): Promise<OverlayHandle> {
  const t = ctx.i18n.t;
  injectStyle(doc);

  let rootEl = doc.getElementById("thermal-overlay-root");
  if (!rootEl) {
    rootEl = doc.createElement("div");
    rootEl.id = "thermal-overlay-root";
    doc.body.appendChild(rootEl);
  }
  rootEl.className = "thm-root";
  rootEl.replaceChildren();

  const overlay = doc.createElement("div");
  overlay.className = "thm-overlay";
  rootEl.appendChild(overlay);

  const canvasEl = doc.createElement("canvas");
  canvasEl.className = "thm-overlay__canvas";
  overlay.appendChild(canvasEl);

  const spotMarkerEl = doc.createElement("div");
  spotMarkerEl.className = "thm-spot";
  overlay.appendChild(spotMarkerEl);

  const readoutEl = doc.createElement("div");
  readoutEl.className = "thm-overlay__readout";
  overlay.appendChild(readoutEl);

  const paletteEl = doc.createElement("div");
  paletteEl.className = "thm-overlay__palette";
  overlay.appendChild(paletteEl);

  let imageData: ImageData | null = null;
  let lastFrame: ThermalFrame | null = null;
  let config: ThermalCameraConfig = { ...DEFAULT_THERMAL_CONFIG };
  let thermalState: ThermalState | null = null;
  // When the current reading was sampled (local clock), plus the read-back
  // and host stamps it carried, so a repeated payload does not refresh the
  // age. The host attaches the agent's `ts_ms` to each payload; the sample
  // time is `min(ts_ms, now)`, so an old entry delivered on mount reads as
  // old and a drone clock running ahead cannot keep a reading live. Without
  // `ts_ms` the arrival time stands in.
  const clock = new SampleClock();
  let spot: SpotMeterState = {
    x: DEFAULT_THERMAL_CONFIG.spotMeter.x,
    y: DEFAULT_THERMAL_CONFIG.spotMeter.y,
    temperatureC: null,
  };

  const currentReadout = (): Readout =>
    readout({
      state: thermalState,
      spotC: spot.temperatureC,
      ageMs: clock.ageMs(Date.now()),
      t,
    });

  function updateReadout(): void {
    const view = currentReadout();
    readoutEl.textContent = view.text;
    readoutEl.classList.toggle(
      "thm-overlay__readout--unavailable",
      view.kind === "unavailable" || view.kind === "unknown",
    );
    readoutEl.classList.toggle(
      "thm-overlay__readout--stale",
      view.kind === "stale",
    );
    const hot =
      view.kind === "spot" &&
      config.alarm.enabled &&
      (spot.temperatureC ?? 0) >= config.alarm.thresholdC;
    readoutEl.classList.toggle("thm-overlay__readout--hot", hot);
    spotMarkerEl.hidden = view.kind !== "spot";

    const palette = paletteText(thermalState, t);
    paletteEl.textContent = palette ?? "";
    paletteEl.hidden = palette === null;
  }

  function drawCurrentFrame(): void {
    if (!lastFrame?.y16 || !imageData) return;
    paintFrame(
      lastFrame,
      {
        palette: config.palette,
        isotherm: config.isotherm,
        fixedRange: config.agc === "fixed" ? config.fixedRange : undefined,
      },
      imageData,
    );
    canvasEl.getContext("2d")?.putImageData(imageData, 0, 0);
  }

  function placeSpotMarker(): void {
    if (!lastFrame) return;
    const rect = canvasEl.getBoundingClientRect();
    const point = frameToCanvas(
      { x: spot.x, y: spot.y },
      { width: rect.width, height: rect.height },
      { width: lastFrame.width, height: lastFrame.height },
    );
    spotMarkerEl.style.left = `${point.clientX}px`;
    spotMarkerEl.style.top = `${point.clientY}px`;
  }

  function ingestFrame(frame: ThermalFrame): void {
    const ts = Number.isFinite(frame.timestampMs) ? frame.timestampMs! : null;
    const hostTs = Number.isFinite(frame.ts_ms) ? frame.ts_ms! : null;
    if (clock.ingest(hostTs, Date.now(), ts ?? hostTs) === null) return;
    lastFrame = frame;

    // The colorized image rides the video leg; the overlay draws the spot
    // marker and readout over it. A lightweight read-back carries the
    // agent-measured spot temperature; a full-frame payload (with y16) is
    // colorized here and metered client-side.
    spot = readSpot(frame, { x: spot.x, y: spot.y });

    if (frame.y16) {
      if (
        canvasEl.width !== frame.width ||
        canvasEl.height !== frame.height ||
        !imageData
      ) {
        canvasEl.width = frame.width;
        canvasEl.height = frame.height;
        const ctx2d = canvasEl.getContext("2d");
        if (ctx2d) imageData = ctx2d.createImageData(frame.width, frame.height);
      }
      drawCurrentFrame();
    }
    placeSpotMarker();
    updateReadout();
  }

  function ingestState(raw: unknown): void {
    const next = parseThermalState(raw);
    if (next === null) return;
    thermalState = next;
    if (!next.connected) {
      // No open session on the agent side means the last frame is stale and
      // no further one is coming; drop the reading so the readout cannot keep
      // showing a temperature for a camera that is gone.
      spot = { x: spot.x, y: spot.y, temperatureC: null };
      clock.reset();
    }
    updateReadout();
  }

  function handleCanvasClick(event: MouseEvent): void {
    // Client-side spot metering needs the Y16 grid. On the lightweight
    // read-back the reticle is fixed by the agent (centre), so a click is a
    // no-op here.
    if (!lastFrame?.y16) return;
    const rect = canvasEl.getBoundingClientRect();
    const point = clientToFrame(
      { clientX: event.clientX - rect.left, clientY: event.clientY - rect.top },
      { width: rect.width, height: rect.height },
      { width: lastFrame.width, height: lastFrame.height },
    );
    spot = {
      x: point.x,
      y: point.y,
      temperatureC: celsiusAt(lastFrame, point.x, point.y),
    };
    placeSpotMarker();
    updateReadout();
  }

  canvasEl.addEventListener("click", handleCanvasClick);
  updateReadout();
  const tick = setInterval(updateReadout, TICK_MS);

  const unsubs: Array<() => void> = [];
  unsubs.push(await ctx.telemetry.subscribe<unknown>("thermal", ingestState));
  unsubs.push(
    await ctx.telemetry.subscribe<ThermalFrame>(
      "camera.thermal.frame",
      (frame) => {
        if (!frame || typeof frame.width !== "number") return;
        ingestFrame(frame);
      },
    ),
  );
  unsubs.push(
    ctx.config.onChange<ThermalCameraConfig>((next) => {
      config = { ...DEFAULT_THERMAL_CONFIG, ...next };
      spot = {
        x: config.spotMeter.x,
        y: config.spotMeter.y,
        temperatureC: spot.temperatureC,
      };
      drawCurrentFrame();
      placeSpotMarker();
      updateReadout();
    }),
  );

  return {
    unmount() {
      clearInterval(tick);
      for (const off of unsubs) off();
      canvasEl.removeEventListener("click", handleCanvasClick);
      overlay.remove();
    },
    getReadout: currentReadout,
  };
}
