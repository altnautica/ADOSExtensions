/**
 * The SIYI Pod control console (node.detail.tab).
 *
 * A vanilla-DOM panel that renders only the controls the connected model
 * supports (from the negotiated capability profile) and wires them to the
 * command emitters. It subscribes to the pod state store for live read-back and
 * capability gating. No framework dependency, so the bundle stays small.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import * as cmd from "./commands";
import { maxZoom, showControl } from "./capability";
import type { PodStateStore } from "./pod-state";
import { GIMBAL_MODES, SENSOR_MODES, type PodState } from "./types";

export interface PanelHandle {
  destroy(): void;
}

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

function button(label: string, onClick: () => void): HTMLButtonElement {
  const b = el("button", { class: "siyi-btn", type: "button" }, [label]);
  b.addEventListener("click", onClick);
  return b;
}

export function mountPanel(
  ctx: PluginContext,
  root: HTMLElement,
  store: PodStateStore,
): PanelHandle {
  const container = el("div", { class: "siyi-panel" });
  root.appendChild(container);

  const render = (state: PodState | null): void => {
    container.textContent = "";
    const t = (k: string) => ctx.i18n.t(k);

    // Header: model + connection + capabilities.
    const header = el("div", { class: "siyi-header" }, [
      el("strong", {}, [state?.model ?? "SIYI Pod"]),
      el("span", { class: "siyi-dim" }, [
        state?.connected ? " connected" : " connecting…",
      ]),
    ]);
    if (state && !state.known) {
      header.appendChild(
        el("div", { class: "siyi-warn" }, ["Unrecognised model — report it to us."]),
      );
    }
    container.appendChild(header);

    // Gimbal.
    if (showControl(state, "gimbal")) {
      const modeSel = el("select", { class: "siyi-sel" }) as HTMLSelectElement;
      for (const m of GIMBAL_MODES) {
        const opt = el("option", { value: m }, [m]) as HTMLOptionElement;
        if (state?.gimbal_mode === m) opt.selected = true;
        modeSel.appendChild(opt);
      }
      modeSel.addEventListener("change", () =>
        void cmd.setGimbalMode(ctx, modeSel.value),
      );
      container.appendChild(
        section(t("settings.gimbal"), [
          modeSel,
          button("Center", () => void cmd.recenter(ctx)),
        ]),
      );
    }

    // Camera.
    const cameraRow: (Node | string)[] = [];
    const sensorSel = el("select", { class: "siyi-sel" }) as HTMLSelectElement;
    for (const m of SENSOR_MODES) {
      const opt = el("option", { value: m }, [m.toUpperCase()]) as HTMLOptionElement;
      if (state?.sensor_mode === m) opt.selected = true;
      sensorSel.appendChild(opt);
    }
    sensorSel.addEventListener("change", () =>
      void cmd.setSensorMode(ctx, sensorSel.value),
    );
    cameraRow.push(sensorSel);
    if (showControl(state, "zoom")) {
      const zoom = el("input", {
        type: "range",
        min: "1",
        max: String(maxZoom(state)),
        step: "0.1",
        value: String(state?.zoom ?? 1),
        class: "siyi-range",
      }) as HTMLInputElement;
      zoom.addEventListener("change", () =>
        void cmd.setZoom(ctx, Number(zoom.value)),
      );
      cameraRow.push(zoom);
    }
    cameraRow.push(button("Photo", () => void cmd.takePhoto(ctx)));
    cameraRow.push(
      button(state?.recording ? "Stop rec" : "Record", () =>
        void cmd.toggleRecord(ctx),
      ),
    );
    container.appendChild(section(t("settings.optics"), cameraRow));

    // Thermal.
    if (showControl(state, "thermal")) {
      const palette = el("input", {
        type: "number",
        min: "0",
        max: "8",
        value: String(state?.palette ?? 0),
        class: "siyi-num",
      }) as HTMLInputElement;
      palette.addEventListener("change", () =>
        void cmd.setPalette(ctx, Number(palette.value)),
      );
      container.appendChild(
        section(t("settings.thermal"), [
          palette,
          button("High gain", () => void cmd.setGain(ctx, true)),
          button("Low gain", () => void cmd.setGain(ctx, false)),
        ]),
      );
    }

    // Laser.
    if (showControl(state, "laser")) {
      const readout =
        state?.laser_range_m != null
          ? `${state.laser_range_m.toFixed(0)} m`
          : "—";
      container.appendChild(
        section(t("settings.laser"), [
          button("Range", () => void cmd.fireLaser(ctx)),
          el("span", { class: "siyi-dim" }, [` ${readout}`]),
        ]),
      );
    }

    // AI track.
    if (showControl(state, "ai_track")) {
      container.appendChild(
        section("Tracking", [
          button(state?.track_active ? "Stop track" : "Track", () =>
            void cmd.setTrackActive(ctx, !state?.track_active),
          ),
        ]),
      );
    }
  };

  const unsub = store.subscribe(render);
  render(store.get());

  return {
    destroy() {
      unsub();
      container.remove();
    },
  };
}

function section(title: string, children: (Node | string)[]): HTMLElement {
  return el("div", { class: "siyi-section" }, [
    el("div", { class: "siyi-section-title" }, [title]),
    el("div", { class: "siyi-row" }, children),
  ]);
}
