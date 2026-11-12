/**
 * Gimbal control panel.
 *
 * Renders pitch / yaw sliders, ROI controls, and a state readout. The panel
 * keeps a tiny in-memory store for the latest `GimbalState`; it never owns
 * business logic for the gimbal itself. Command emission goes through
 * `commands.ts`, which routes to the host's `ctx.command.send`.
 *
 * The host frame is sandboxed without `allow-forms`, so the ROI controls are
 * plain buttons and Enter in a field, never a form submission.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import {
  sendPitchYaw,
  sendRoiLocation,
  sendRoiNone,
} from "./commands";
import {
  DEFAULT_LIMITS,
  type AxisLimits,
  type GimbalMode,
  type GimbalState,
  type PanelOptions,
  type RoiTarget,
} from "./types";

/** The gimbal feed runs at 5 Hz; a second of silence means it stopped. */
export const STALE_AFTER_MS = 1000;
const TICK_MS = 1000;
/** Below this a `timestampMs` is not wall-clock milliseconds. */
const EPOCH_MS_FLOOR = 1e12;

const MODE_KEYS: Record<GimbalMode, string> = {
  manual: "mode.manual",
  "roi-lock": "mode.roiLock",
  "follow-me": "mode.followMe",
  neutral: "mode.neutral",
};

type Translate = (key: string, params?: Record<string, string | number>) => string;

export interface PanelHandle {
  /** Replace the live gimbal state and re-render. */
  setState(state: GimbalState | null): void;
  /** Read the latest live state. */
  getState(): GimbalState | null;
  /** Programmatically trigger a slider change for tests. */
  setSlider(axis: "pitch" | "yaw", value: number): void;
  /** Programmatically send an ROI lock for tests. */
  submitRoi(target: RoiTarget): Promise<void>;
  /** Programmatically click the release-ROI button for tests. */
  releaseRoi(): Promise<void>;
  destroy(): void;
}

export function mountPanel(
  ctx: PluginContext,
  rootEl: HTMLElement,
  opts: PanelOptions = {},
): PanelHandle {
  const t: Translate = (key, params) => ctx.i18n.t(key, params);
  const limits: AxisLimits = opts.limits ?? DEFAULT_LIMITS;
  let state: GimbalState | null = null;
  let receivedAt = 0;
  // The ROI request awaiting confirmation from the telemetry mode.
  let pendingRoi: "lock" | "release" | null = null;
  const dragging = { pitch: false, yaw: false };
  const deg = (value: number): string => t("unit.deg", { value: value.toFixed(1) });

  const dom = render(rootEl, limits, t);

  function ageMs(now: number): number {
    if (!state) return Infinity;
    const sinceReceipt = now - receivedAt;
    return state.timestampMs > EPOCH_MS_FLOOR
      ? Math.max(sinceReceipt, now - state.timestampMs)
      : sinceReceipt;
  }

  function isFresh(): boolean {
    return ageMs(Date.now()) < STALE_AFTER_MS;
  }

  function refreshReadout(): void {
    const age = ageMs(Date.now());
    if (state && age < STALE_AFTER_MS) {
      dom.statePitch.textContent = deg(state.pitchDeg);
      dom.stateYaw.textContent = deg(state.yawDeg);
      dom.stateRoll.textContent = deg(state.rollDeg);
      dom.stateMode.textContent = t(MODE_KEYS[state.mode] ?? "mode.neutral");
      dom.empty.hidden = true;
      dom.readout.hidden = false;
    } else {
      dom.empty.textContent = state
        ? t("panel.notReporting", { age: Math.floor(age / 1000) })
        : t("panel.awaitingState");
      dom.empty.hidden = false;
      dom.readout.hidden = true;
    }
  }

  function showSlider(axis: "pitch" | "yaw", value: number): void {
    (axis === "pitch" ? dom.pitchValue : dom.yawValue).textContent = deg(value);
  }

  // Sliders follow the live angles while untouched, so a release sends the
  // other axis where the gimbal actually is rather than where its slider
  // started.
  function seedSliders(): void {
    if (!state || !isFresh()) return;
    for (const axis of ["pitch", "yaw"] as const) {
      if (dragging[axis]) continue;
      const value = axis === "pitch" ? state.pitchDeg : state.yawDeg;
      (axis === "pitch" ? dom.pitch : dom.yaw).value = String(value);
      showSlider(axis, value);
    }
  }

  function confirmRoi(): void {
    if (!state || !isFresh()) return;
    if (pendingRoi === "lock" && state.mode === "roi-lock") {
      dom.roiStatus.textContent = t("roi.locked");
      pendingRoi = null;
    } else if (pendingRoi === "release" && state.mode !== "roi-lock") {
      dom.roiStatus.textContent = t("roi.released");
      pendingRoi = null;
    }
  }

  // Dragging only moves the label; the command goes once, on release, so a
  // drag is one gimbal move rather than a write per pixel.
  async function commitSlider(axis: "pitch" | "yaw", value: number): Promise<void> {
    dragging[axis] = false;
    showSlider(axis, value);
    const pitchDeg = axis === "pitch" ? value : numberValue(dom.pitch);
    const yawDeg = axis === "yaw" ? value : numberValue(dom.yaw);
    try {
      await sendPitchYaw(ctx, { pitchDeg, yawDeg });
      dom.pointStatus.textContent = "";
    } catch {
      dom.pointStatus.textContent = t("manual.pointDenied");
    }
  }

  for (const axis of ["pitch", "yaw"] as const) {
    const input = axis === "pitch" ? dom.pitch : dom.yaw;
    input.addEventListener("input", () => {
      dragging[axis] = true;
      showSlider(axis, numberValue(input));
    });
    input.addEventListener("change", () => void commitSlider(axis, numberValue(input)));
  }

  async function submitRoi(target: RoiTarget): Promise<void> {
    try {
      await sendRoiLocation(ctx, target);
    } catch {
      pendingRoi = null;
      dom.roiStatus.textContent = t("roi.lockDenied");
      return;
    }
    if (state && isFresh()) {
      pendingRoi = "lock";
      dom.roiStatus.textContent = t("roi.sent");
    } else {
      pendingRoi = null;
      dom.roiStatus.textContent = t("roi.noGimbal");
    }
  }

  async function releaseRoi(): Promise<void> {
    try {
      await sendRoiNone(ctx);
    } catch {
      pendingRoi = null;
      dom.roiStatus.textContent = t("roi.releaseDenied");
      return;
    }
    if (state && isFresh()) {
      pendingRoi = "release";
      dom.roiStatus.textContent = t("roi.releaseSent");
    } else {
      pendingRoi = null;
      dom.roiStatus.textContent = t("roi.noGimbal");
    }
  }

  function lockFromFields(): void {
    void submitRoi({
      latDeg: numberValue(dom.roiLat),
      lonDeg: numberValue(dom.roiLon),
      altM: numberValue(dom.roiAlt),
    });
  }

  dom.roiLock.addEventListener("click", lockFromFields);
  for (const field of [dom.roiLat, dom.roiLon, dom.roiAlt]) {
    field.addEventListener("keydown", (e) => {
      if (e.key !== "Enter") return;
      e.preventDefault();
      lockFromFields();
    });
  }
  dom.roiRelease.addEventListener("click", () => void releaseRoi());

  const tick = setInterval(refreshReadout, TICK_MS);
  refreshReadout();

  return {
    setState(next) {
      state = next;
      receivedAt = Date.now();
      refreshReadout();
      seedSliders();
      confirmRoi();
    },
    getState() {
      return state;
    },
    setSlider(axis, value) {
      const input = axis === "pitch" ? dom.pitch : dom.yaw;
      input.value = String(value);
      void commitSlider(axis, value);
    },
    submitRoi,
    releaseRoi,
    destroy() {
      clearInterval(tick);
      rootEl.innerHTML = "";
    },
  };
}

interface DomRefs {
  pitch: HTMLInputElement;
  yaw: HTMLInputElement;
  pitchValue: HTMLElement;
  yawValue: HTMLElement;
  pointStatus: HTMLElement;
  roiLat: HTMLInputElement;
  roiLon: HTMLInputElement;
  roiAlt: HTMLInputElement;
  roiLock: HTMLButtonElement;
  roiRelease: HTMLButtonElement;
  roiStatus: HTMLElement;
  statePitch: HTMLElement;
  stateYaw: HTMLElement;
  stateRoll: HTMLElement;
  stateMode: HTMLElement;
  readout: HTMLElement;
  empty: HTMLElement;
}

function render(rootEl: HTMLElement, limits: AxisLimits, t: Translate): DomRefs {
  rootEl.innerHTML = "";
  rootEl.classList.add("agm-root");

  const empty = el("p", "agm-empty", t("panel.awaitingState"));
  rootEl.appendChild(empty);

  const readout = el("section", "agm-readout");
  readout.appendChild(el("h2", "agm-section-title", t("state.title")));
  const statePitch = el("span", "agm-readout-value", "");
  const stateYaw = el("span", "agm-readout-value", "");
  const stateRoll = el("span", "agm-readout-value", "");
  const stateMode = el("span", "agm-readout-value agm-mode", "");
  readout.appendChild(kv(t("state.pitch"), statePitch));
  readout.appendChild(kv(t("state.yaw"), stateYaw));
  readout.appendChild(kv(t("state.roll"), stateRoll));
  readout.appendChild(kv(t("state.mode"), stateMode));
  rootEl.appendChild(readout);

  const manual = el("section", "agm-manual");
  manual.appendChild(el("h2", "agm-section-title", t("manual.title")));
  const pitch = slider("agm-pitch", limits.pitchMinDeg, limits.pitchMaxDeg, 0);
  const yaw = slider("agm-yaw", limits.yawMinDeg, limits.yawMaxDeg, 0);
  const zero = t("unit.deg", { value: (0).toFixed(1) });
  const pitchValue = el("span", "agm-slider-value", zero);
  const yawValue = el("span", "agm-slider-value", zero);
  manual.appendChild(sliderRow(t("manual.pitch"), pitch, pitchValue));
  manual.appendChild(sliderRow(t("manual.yaw"), yaw, yawValue));
  const pointStatus = el("p", "agm-roi-status", "");
  manual.appendChild(pointStatus);
  rootEl.appendChild(manual);

  const roiSection = el("section", "agm-roi");
  roiSection.appendChild(el("h2", "agm-section-title", t("roi.title")));
  const roiFields = el("div", "agm-roi-form");
  const roiLat = numberInput("agm-roi-lat", 0);
  const roiLon = numberInput("agm-roi-lon", 0);
  const roiAlt = numberInput("agm-roi-alt", 50);
  roiFields.appendChild(labeled(t("roi.lat"), roiLat));
  roiFields.appendChild(labeled(t("roi.lon"), roiLon));
  roiFields.appendChild(labeled(t("roi.alt"), roiAlt));
  const roiLock = button("agm-roi-submit", t("roi.lock"));
  roiFields.appendChild(roiLock);
  const roiRelease = button("agm-roi-release", t("roi.release"));
  roiFields.appendChild(roiRelease);
  roiSection.appendChild(roiFields);
  const roiStatus = el("p", "agm-roi-status", "");
  roiStatus.setAttribute("role", "status");
  roiSection.appendChild(roiStatus);
  rootEl.appendChild(roiSection);

  return {
    pitch,
    yaw,
    pitchValue,
    yawValue,
    pointStatus,
    roiLat,
    roiLon,
    roiAlt,
    roiLock,
    roiRelease,
    roiStatus,
    statePitch,
    stateYaw,
    stateRoll,
    stateMode,
    readout,
    empty,
  };
}

function el(
  tag: keyof HTMLElementTagNameMap,
  className: string,
  text?: string,
): HTMLElement {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function button(className: string, text: string): HTMLButtonElement {
  const node = document.createElement("button");
  node.type = "button";
  node.className = className;
  node.textContent = text;
  return node;
}

function kv(label: string, valueEl: HTMLElement): HTMLElement {
  const row = document.createElement("div");
  row.className = "agm-row";
  const k = document.createElement("span");
  k.className = "agm-key";
  k.textContent = label;
  row.appendChild(k);
  row.appendChild(valueEl);
  return row;
}

function slider(
  className: string,
  min: number,
  max: number,
  value: number,
): HTMLInputElement {
  const input = document.createElement("input");
  input.type = "range";
  input.className = className;
  input.min = String(min);
  input.max = String(max);
  input.step = "0.5";
  input.value = String(value);
  return input;
}

function sliderRow(
  label: string,
  input: HTMLInputElement,
  valueEl: HTMLElement,
): HTMLElement {
  const row = document.createElement("div");
  row.className = "agm-slider-row";
  const labelEl = document.createElement("label");
  labelEl.textContent = label;
  labelEl.appendChild(input);
  row.appendChild(labelEl);
  row.appendChild(valueEl);
  return row;
}

function numberInput(className: string, value: number): HTMLInputElement {
  const input = document.createElement("input");
  input.type = "number";
  input.className = className;
  input.step = "any";
  input.value = String(value);
  return input;
}

function labeled(label: string, input: HTMLInputElement): HTMLElement {
  const wrap = document.createElement("label");
  wrap.className = "agm-roi-field";
  const text = document.createElement("span");
  text.textContent = label;
  wrap.appendChild(text);
  wrap.appendChild(input);
  return wrap;
}

function numberValue(input: HTMLInputElement): number {
  const n = Number(input.value);
  return Number.isFinite(n) ? n : 0;
}
