/**
 * The SIYI Pod control console (node.detail.tab and cockpit.panel).
 *
 * A vanilla-DOM panel that renders only the controls the connected model
 * supports (from the negotiated capability profile) and wires them to the
 * command emitters. The controls are built once per capability set and the
 * read-back is written into them in place, so a 5 Hz feed never steals focus
 * or overwrites a control the operator is using. A 1 Hz tick ages the
 * read-back: a feed that stops reads "No pod data" and its values are dropped.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import * as cmd from "./commands";
import { maxZoom, showControl } from "./capability";
import { dataAgeS, isLive, rangeReading, type PodStateStore } from "./pod-state";
import { GIMBAL_MODES, PALETTE_CODES, type PodState } from "./types";

export interface PanelHandle {
  destroy(): void;
  /** Hide the console (the video.overlay slot shows only the HUD). */
  hide(): void;
}

/** How long the Range button waits for its confirming second press. */
export const RANGE_CONFIRM_MS = 5_000;

const ERROR_CODES: Record<string, true> = { refused: true, timeout: true, disposed: true };

function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  attrs: Partial<Record<string, string>> = {},
  children: (Node | string)[] = [],
): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v !== undefined) node.setAttribute(k, v);
  }
  for (const child of children) {
    node.appendChild(typeof child === "string" ? document.createTextNode(child) : child);
  }
  return node;
}

function button(label: string, onClick: () => void, testid?: string): HTMLButtonElement {
  const b = el("button", { class: "siyi-btn", type: "button", "data-testid": testid }, [label]);
  b.addEventListener("click", onClick);
  return b;
}

function section(title: string, children: (Node | string)[]): HTMLElement {
  return el("div", { class: "siyi-section" }, [
    el("div", { class: "siyi-section-title" }, [title]),
    el("div", { class: "siyi-row" }, children),
  ]);
}

/** The controls built for one capability set; updated in place. */
interface Controls {
  mode?: HTMLSelectElement;
  zoom?: HTMLInputElement;
  record: HTMLButtonElement;
  palette?: HTMLSelectElement;
  rangeBtn?: HTMLButtonElement;
  rangeReadout?: HTMLSpanElement;
  laserNote?: HTMLDivElement;
}

export function mountPanel(
  ctx: PluginContext,
  root: HTMLElement,
  store: PodStateStore,
): PanelHandle {
  const t = (k: string, p?: Record<string, string | number>) => ctx.i18n.t(k, p);

  const container = el("div", { class: "siyi-panel" });
  const model = el("strong");
  const link = el("span", { class: "siyi-link", "data-testid": "siyi-link" });
  const age = el("span", { class: "siyi-dim", "data-testid": "siyi-age" });
  const unknown = el("div", { class: "siyi-warn" }, [t("panel.unknownModel")]);
  const header = el("div", { class: "siyi-header" }, [model, " ", link, " ", age, unknown]);
  const body = el("div", { class: "siyi-body", "data-testid": "siyi-body" });
  container.append(header, body);
  root.appendChild(container);
  // A refused or failed command (operator denial, no agent link) is shown
  // here as a sentence instead of vanishing.
  const status = el("div", { class: "siyi-warn", "data-testid": "siyi-command-status" });
  root.appendChild(status);
  const run = (pending: Promise<unknown>): void => {
    status.textContent = "";
    pending.catch((err: unknown) => {
      const code = (err as { code?: unknown } | null)?.code;
      status.textContent =
        typeof code === "string" && Object.hasOwn(ERROR_CODES, code) ? t(`error.${code}`) : t("error.failed");
    });
  };

  // -- Range confirmation ------------------------------------------------
  let confirmTimer: number | null = null;
  const resetRange = (): void => {
    clearTimeout(confirmTimer ?? undefined);
    confirmTimer = null;
    if (controls?.rangeBtn) controls.rangeBtn.textContent = t("panel.range");
  };
  const onRange = (): void => {
    const btn = controls?.rangeBtn;
    if (!btn || btn.disabled) return;
    if (confirmTimer === null) {
      btn.textContent = t("panel.rangeConfirm");
      confirmTimer = window.setTimeout(resetRange, RANGE_CONFIRM_MS);
      return;
    }
    resetRange();
    run(cmd.fireLaser(ctx));
  };

  // -- Build once per capability set ---------------------------------------
  let controls: Controls | null = null;
  let builtFor: string | null = null;

  const build = (state: PodState | null): Controls => {
    body.textContent = "";
    resetRange();
    const built: Controls = {
      record: button(t("panel.record"), () => run(cmd.toggleRecord(ctx))),
    };

    if (showControl(state, "gimbal")) {
      const mode = el("select", {
        class: "siyi-sel",
        "data-testid": "siyi-mode",
        "aria-label": t("panel.mode"),
      });
      for (const m of GIMBAL_MODES) mode.appendChild(el("option", { value: m }, [t(`mode.${m}`)]));
      mode.addEventListener("change", () => run(cmd.setGimbalMode(ctx, mode.value)));
      built.mode = mode;
      body.appendChild(
        section(t("settings.gimbal"), [
          mode,
          button(t("panel.recenter"), () => run(cmd.recenter(ctx))),
        ]),
      );
    }

    // A multi-sensor pod serves two concurrent streams (main + sub) and the
    // cockpit switches the view between them; the panel keeps the per-sensor
    // controls (zoom, photo, record) here.
    const cameraRow: Node[] = [];
    if (showControl(state, "zoom")) {
      const zoom = el("input", {
        type: "range",
        min: "1",
        max: String(maxZoom(state)),
        step: "0.1",
        class: "siyi-range",
        "aria-label": t("panel.zoom"),
      });
      zoom.addEventListener("change", () => run(cmd.setZoom(ctx, Number(zoom.value))));
      built.zoom = zoom;
      cameraRow.push(zoom);
    }
    cameraRow.push(button(t("panel.photo"), () => run(cmd.takePhoto(ctx))), built.record);
    body.appendChild(section(t("settings.optics"), cameraRow));

    if (showControl(state, "thermal")) {
      const palette = el("select", {
        class: "siyi-sel",
        "data-testid": "siyi-palette",
        "aria-label": t("panel.palette"),
      });
      for (const code of PALETTE_CODES) {
        palette.appendChild(el("option", { value: String(code) }, [t(`palette.${code}`)]));
      }
      palette.addEventListener("change", () => run(cmd.setPalette(ctx, Number(palette.value))));
      built.palette = palette;
      body.appendChild(
        section(t("settings.thermal"), [
          palette,
          button(t("panel.highGain"), () => run(cmd.setGain(ctx, true))),
          button(t("panel.lowGain"), () => run(cmd.setGain(ctx, false))),
        ]),
      );
    }

    if (showControl(state, "laser")) {
      built.rangeBtn = button(t("panel.range"), onRange, "siyi-range-btn");
      built.rangeReadout = el("span", { class: "siyi-dim", "data-testid": "siyi-range" });
      built.laserNote = el("div", { class: "siyi-warn", "data-testid": "siyi-laser-note" }, [
        t("panel.laserDisarmed"),
      ]);
      const laser = section(t("settings.laser"), [built.rangeBtn, built.rangeReadout]);
      laser.appendChild(built.laserNote);
      body.appendChild(laser);
    }
    return built;
  };

  // -- Write the read-back in place ------------------------------------------
  const setIfIdle = (control: HTMLInputElement | HTMLSelectElement | undefined, value: string) => {
    if (!control || document.activeElement === control || control.value === value) return;
    control.value = value;
  };

  const update = (): void => {
    const state = store.get();
    const now = Date.now();
    const key = state ? `${state.known}|${JSON.stringify(state.capabilities)}` : "";
    if (controls === null || key !== builtFor) {
      controls = build(state);
      builtFor = key;
    }
    const live = isLive(store, now);
    const answering = live && state !== null && state.connected && state.link_ok;

    model.textContent = state?.model ?? t("panel.defaultModel");
    unknown.hidden = !(state && !state.known);
    if (state && !live) {
      link.textContent = t("panel.noData");
      age.textContent = t("panel.lastUpdate", { s: dataAgeS(store, now) ?? 0 });
    } else {
      link.textContent = !state?.connected
        ? t("panel.connecting")
        : state.link_ok
          ? t("panel.connected")
          : t("panel.notAnswering");
      age.textContent = "";
    }
    const lost = state !== null && !answering;
    body.classList.toggle("siyi-lost", lost);
    body.style.opacity = lost ? "0.45" : "";

    const c = controls;
    if (live && state) {
      setIfIdle(c.mode, state.gimbal_mode);
      setIfIdle(c.zoom, String(state.zoom ?? 1));
      if (c.palette && document.activeElement !== c.palette) {
        c.palette.value = state.palette === null ? "" : String(state.palette);
      }
      c.record.textContent = state.recording ? t("panel.stopRecord") : t("panel.record");
    }
    if (c.rangeBtn && c.rangeReadout && c.laserNote) {
      const armed = state?.laser_enabled === true;
      c.laserNote.hidden = armed;
      c.rangeBtn.disabled = !armed || !answering;
      if (c.rangeBtn.disabled) resetRange();
      const range = live ? rangeReading(state, now) : null;
      c.rangeReadout.textContent = range
        ? t("panel.rangeReading", { m: range.range_m.toFixed(0), s: range.age_s })
        : "—";
    }
  };

  const unsub = store.subscribe(update);
  update();
  const tick = window.setInterval(update, 1_000);

  return {
    destroy() {
      unsub();
      clearInterval(tick);
      resetRange();
      container.remove();
      status.remove();
    },
    hide() {
      container.style.display = "none";
      status.style.display = "none";
    },
  };
}
