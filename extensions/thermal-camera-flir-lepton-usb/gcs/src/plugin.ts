/**
 * Plugin entry point. Built into ``plugin.bundle.js`` and loaded by
 * the GCS plugin host inside a sandboxed iframe. The plugin renders
 * a thermal video overlay on top of the host's video pane and exposes
 * palette, spot meter, and isotherm controls in a compact action rail.
 */

import { definePlugin } from "@altnautica/plugin-sdk";

import { listPalettes } from "./palettes";
import { paintFrame } from "./render";
import { celsiusAt, clientToFrame, frameToCanvas } from "./spotMeter";
import {
  DEFAULT_THERMAL_CONFIG,
  type PaletteName,
  type SpotMeterState,
  type ThermalCameraConfig,
  type ThermalFrame,
} from "./types";

let rootEl: HTMLElement | null = null;
let canvasEl: HTMLCanvasElement | null = null;
let spotMarkerEl: HTMLDivElement | null = null;
let readoutEl: HTMLDivElement | null = null;
let imageData: ImageData | null = null;
let lastFrame: ThermalFrame | null = null;
let config: ThermalCameraConfig = { ...DEFAULT_THERMAL_CONFIG };
let spot: SpotMeterState = {
  x: DEFAULT_THERMAL_CONFIG.spotMeter.x,
  y: DEFAULT_THERMAL_CONFIG.spotMeter.y,
  temperatureC: null,
};

definePlugin({
  id: "com.altnautica.thermal-flir-lepton-usb",
  version: "1.0.0",
  async mount(ctx) {
    mountDom();
    renderActionRail();

    await ctx.telemetry.subscribe<ThermalFrame>(
      "camera.thermal.frame",
      (frame) => {
        if (!frame || typeof frame.width !== "number") return;
        ingestFrame(frame);
      },
    );
    ctx.config.onChange<ThermalCameraConfig>((next) => {
      config = { ...DEFAULT_THERMAL_CONFIG, ...next };
      spot = {
        x: config.spotMeter.x,
        y: config.spotMeter.y,
        temperatureC: spot.temperatureC,
      };
      drawCurrentFrame();
      placeSpotMarker();
    });
  },
});

function mountDom(): void {
  rootEl = document.getElementById("thermal-overlay-root");
  if (!rootEl) {
    rootEl = document.createElement("div");
    rootEl.id = "thermal-overlay-root";
    rootEl.className = "thm-root";
    document.body.appendChild(rootEl);
  }

  const overlay = document.createElement("div");
  overlay.className = "thm-overlay";
  rootEl.appendChild(overlay);

  canvasEl = document.createElement("canvas");
  canvasEl.className = "thm-overlay__canvas";
  overlay.appendChild(canvasEl);

  spotMarkerEl = document.createElement("div");
  spotMarkerEl.className = "thm-spot";
  overlay.appendChild(spotMarkerEl);

  readoutEl = document.createElement("div");
  readoutEl.className = "thm-overlay__readout";
  readoutEl.textContent = "Awaiting thermal frames...";
  overlay.appendChild(readoutEl);

  canvasEl.addEventListener("click", (event) => handleCanvasClick(event));
}

function renderActionRail(): void {
  if (!rootEl) return;
  const rail = document.createElement("div");
  rail.className = "thm-action-rail";

  const select = document.createElement("select");
  select.className = "thm-action-rail__button";
  for (const name of listPalettes()) {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    if (name === config.palette) opt.selected = true;
    select.appendChild(opt);
  }
  select.addEventListener("change", () => {
    config = { ...config, palette: select.value as PaletteName };
    drawCurrentFrame();
  });
  rail.appendChild(select);

  rootEl.appendChild(rail);
}

function ingestFrame(frame: ThermalFrame): void {
  lastFrame = frame;
  if (!canvasEl) return;
  if (
    canvasEl.width !== frame.width ||
    canvasEl.height !== frame.height ||
    !imageData
  ) {
    canvasEl.width = frame.width;
    canvasEl.height = frame.height;
    const ctx2d = canvasEl.getContext("2d");
    if (!ctx2d) return;
    imageData = ctx2d.createImageData(frame.width, frame.height);
  }
  drawCurrentFrame();
  spot = {
    x: spot.x,
    y: spot.y,
    temperatureC: celsiusAt(frame, spot.x, spot.y),
  };
  placeSpotMarker();
  updateReadout();
}

function drawCurrentFrame(): void {
  if (!canvasEl || !lastFrame || !imageData) return;
  paintFrame(
    lastFrame,
    {
      palette: config.palette,
      isotherm: config.isotherm,
      fixedRange:
        config.agc === "fixed" ? config.fixedRange : undefined,
    },
    imageData,
  );
  const ctx2d = canvasEl.getContext("2d");
  if (!ctx2d) return;
  ctx2d.putImageData(imageData, 0, 0);
}

function placeSpotMarker(): void {
  if (!canvasEl || !spotMarkerEl || !lastFrame) return;
  const rect = canvasEl.getBoundingClientRect();
  const point = frameToCanvas(
    { x: spot.x, y: spot.y },
    { width: rect.width, height: rect.height },
    { width: lastFrame.width, height: lastFrame.height },
  );
  spotMarkerEl.style.left = `${point.clientX}px`;
  spotMarkerEl.style.top = `${point.clientY}px`;
}

function updateReadout(): void {
  if (!readoutEl) return;
  if (spot.temperatureC === null) {
    readoutEl.textContent = "Awaiting thermal frames...";
    return;
  }
  readoutEl.textContent = `Spot ${spot.temperatureC.toFixed(1)} °C`;
  if (config.alarm.enabled && spot.temperatureC >= config.alarm.thresholdC) {
    readoutEl.classList.add("thm-overlay__readout--hot");
  } else {
    readoutEl.classList.remove("thm-overlay__readout--hot");
  }
}

function handleCanvasClick(event: MouseEvent): void {
  if (!canvasEl || !lastFrame) return;
  const rect = canvasEl.getBoundingClientRect();
  const point = clientToFrame(
    {
      clientX: event.clientX - rect.left,
      clientY: event.clientY - rect.top,
    },
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

export const __test = {
  ingestFrame,
  getSpot: (): SpotMeterState => ({ ...spot }),
  getConfig: (): ThermalCameraConfig => ({ ...config }),
};
