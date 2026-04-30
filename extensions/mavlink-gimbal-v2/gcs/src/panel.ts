/**
 * Gimbal control panel.
 *
 * Renders pitch / yaw / roll sliders, an ROI form, and a state
 * readout. The panel keeps a tiny in-memory store for the latest
 * `GimbalState`; it never owns business logic for the gimbal itself.
 * Command emission goes through `commands.ts`, which routes to the
 * host's `ctx.command.send`.
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
  type GimbalState,
  type PanelOptions,
  type RoiTarget,
} from "./types";

export interface PanelHandle {
  /** Replace the live gimbal state and re-render. */
  setState(state: GimbalState | null): void;
  /** Read the latest live state. */
  getState(): GimbalState | null;
  /** Programmatically trigger a slider change for tests. */
  setSlider(axis: "pitch" | "yaw" | "roll", value: number): void;
  /** Programmatically submit the ROI form for tests. */
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
  const limits: AxisLimits = opts.limits ?? DEFAULT_LIMITS;
  const targets = {
    targetSystem: opts.vehicleSystemId,
    targetComponent: opts.vehicleComponentId,
  };
  let state: GimbalState | null = null;

  const dom = render(rootEl, limits);

  function refreshReadout(): void {
    if (state) {
      dom.statePitch.textContent = formatDeg(state.pitchDeg);
      dom.stateYaw.textContent = formatDeg(state.yawDeg);
      dom.stateRoll.textContent = formatDeg(state.rollDeg);
      dom.stateMode.textContent = state.mode;
      dom.empty.hidden = true;
      dom.readout.hidden = false;
    } else {
      dom.empty.hidden = false;
      dom.readout.hidden = true;
    }
  }

  async function onSliderChange(
    axis: "pitch" | "yaw" | "roll",
    value: number,
  ): Promise<void> {
    if (axis === "pitch") dom.pitchValue.textContent = formatDeg(value);
    if (axis === "yaw") dom.yawValue.textContent = formatDeg(value);
    if (axis === "roll") dom.rollValue.textContent = formatDeg(value);
    const pitchDeg =
      axis === "pitch" ? value : numberValue(dom.pitch);
    const yawDeg = axis === "yaw" ? value : numberValue(dom.yaw);
    try {
      await sendPitchYaw(ctx, { pitchDeg, yawDeg, ...targets });
    } catch {
      // Host may deny per-action consent. The slider visual stays at
      // the operator-set value; the next telemetry tick will reset it
      // if the agent did not move.
    }
  }

  dom.pitch.addEventListener("input", (e) =>
    void onSliderChange("pitch", numberValue(e.currentTarget as HTMLInputElement)),
  );
  dom.yaw.addEventListener("input", (e) =>
    void onSliderChange("yaw", numberValue(e.currentTarget as HTMLInputElement)),
  );
  dom.roll.addEventListener("input", (e) =>
    void onSliderChange("roll", numberValue(e.currentTarget as HTMLInputElement)),
  );

  async function submitRoi(target: RoiTarget): Promise<void> {
    try {
      await sendRoiLocation(ctx, target, targets);
      dom.roiStatus.textContent = `Locked on ${target.latDeg}, ${target.lonDeg}, ${target.altM} m`;
    } catch {
      dom.roiStatus.textContent = "ROI lock denied by host";
    }
  }

  async function releaseRoi(): Promise<void> {
    try {
      await sendRoiNone(ctx, targets);
      dom.roiStatus.textContent = "ROI released";
    } catch {
      dom.roiStatus.textContent = "Release denied by host";
    }
  }

  dom.roiForm.addEventListener("submit", (e) => {
    e.preventDefault();
    const lat = numberValue(dom.roiLat);
    const lon = numberValue(dom.roiLon);
    const alt = numberValue(dom.roiAlt);
    void submitRoi({ latDeg: lat, lonDeg: lon, altM: alt });
  });
  dom.roiRelease.addEventListener("click", () => void releaseRoi());

  refreshReadout();

  return {
    setState(next) {
      state = next;
      refreshReadout();
    },
    getState() {
      return state;
    },
    setSlider(axis, value) {
      const el =
        axis === "pitch" ? dom.pitch : axis === "yaw" ? dom.yaw : dom.roll;
      el.value = String(value);
      void onSliderChange(axis, value);
    },
    submitRoi,
    releaseRoi,
    destroy() {
      rootEl.innerHTML = "";
    },
  };
}

interface DomRefs {
  pitch: HTMLInputElement;
  yaw: HTMLInputElement;
  roll: HTMLInputElement;
  pitchValue: HTMLElement;
  yawValue: HTMLElement;
  rollValue: HTMLElement;
  roiForm: HTMLFormElement;
  roiLat: HTMLInputElement;
  roiLon: HTMLInputElement;
  roiAlt: HTMLInputElement;
  roiRelease: HTMLButtonElement;
  roiStatus: HTMLElement;
  statePitch: HTMLElement;
  stateYaw: HTMLElement;
  stateRoll: HTMLElement;
  stateMode: HTMLElement;
  readout: HTMLElement;
  empty: HTMLElement;
}

function render(rootEl: HTMLElement, limits: AxisLimits): DomRefs {
  rootEl.innerHTML = "";
  rootEl.classList.add("agm-root");

  const empty = el("p", "agm-empty", "Awaiting gimbal state...");
  rootEl.appendChild(empty);

  const readout = el("section", "agm-readout");
  readout.appendChild(el("h2", "agm-section-title", "State"));
  const statePitch = el("span", "agm-readout-value", "0.0 deg");
  const stateYaw = el("span", "agm-readout-value", "0.0 deg");
  const stateRoll = el("span", "agm-readout-value", "0.0 deg");
  const stateMode = el("span", "agm-readout-value", "neutral");
  readout.appendChild(kv("Pitch", statePitch));
  readout.appendChild(kv("Yaw", stateYaw));
  readout.appendChild(kv("Roll", stateRoll));
  readout.appendChild(kv("Mode", stateMode));
  rootEl.appendChild(readout);

  const manual = el("section", "agm-manual");
  manual.appendChild(el("h2", "agm-section-title", "Manual control"));
  const pitch = slider(
    "agm-pitch",
    limits.pitchMinDeg,
    limits.pitchMaxDeg,
    0,
  );
  const yaw = slider("agm-yaw", limits.yawMinDeg, limits.yawMaxDeg, 0);
  const roll = slider("agm-roll", limits.rollMinDeg, limits.rollMaxDeg, 0);
  const pitchValue = el("span", "agm-slider-value", "0 deg");
  const yawValue = el("span", "agm-slider-value", "0 deg");
  const rollValue = el("span", "agm-slider-value", "0 deg");
  manual.appendChild(sliderRow("Pitch", pitch, pitchValue));
  manual.appendChild(sliderRow("Yaw", yaw, yawValue));
  manual.appendChild(sliderRow("Roll", roll, rollValue));
  rootEl.appendChild(manual);

  const roiSection = el("section", "agm-roi");
  roiSection.appendChild(el("h2", "agm-section-title", "Region of interest"));
  const roiForm = document.createElement("form");
  roiForm.className = "agm-roi-form";
  const roiLat = numberInput("agm-roi-lat", "Latitude", 0);
  const roiLon = numberInput("agm-roi-lon", "Longitude", 0);
  const roiAlt = numberInput("agm-roi-alt", "Altitude (m AGL)", 50);
  roiForm.appendChild(labeled("Lat", roiLat));
  roiForm.appendChild(labeled("Lon", roiLon));
  roiForm.appendChild(labeled("Alt", roiAlt));
  const submit = document.createElement("button");
  submit.type = "submit";
  submit.className = "agm-roi-submit";
  submit.textContent = "Lock on target";
  roiForm.appendChild(submit);
  const roiRelease = document.createElement("button");
  roiRelease.type = "button";
  roiRelease.className = "agm-roi-release";
  roiRelease.textContent = "Release ROI";
  roiForm.appendChild(roiRelease);
  roiSection.appendChild(roiForm);
  const roiStatus = el("p", "agm-roi-status", "");
  roiSection.appendChild(roiStatus);
  rootEl.appendChild(roiSection);

  return {
    pitch,
    yaw,
    roll,
    pitchValue,
    yawValue,
    rollValue,
    roiForm,
    roiLat,
    roiLon,
    roiAlt,
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

function numberInput(
  className: string,
  ariaLabel: string,
  value: number,
): HTMLInputElement {
  const input = document.createElement("input");
  input.type = "number";
  input.className = className;
  input.step = "any";
  input.value = String(value);
  input.setAttribute("aria-label", ariaLabel);
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

function formatDeg(value: number): string {
  return `${value.toFixed(1)} deg`;
}
