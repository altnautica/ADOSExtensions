/**
 * Hello hybrid plugin — GCS half.
 *
 * The plain settings (rate, greeting, detector) are declared in the manifest
 * as native `contributes.parameters`, so the GCS renders those controls for
 * you — this iframe does NOT re-implement them. This bundle backs the
 * `node.detail.tab` contribution and renders only what a native control can't:
 * a live read-back of the agent half's `hello.state` event.
 *
 * The same bundle would back any other iframe slot the plugin contributes
 * (an overlay, a notification channel); tell them apart at runtime by the
 * host-prop stream each slot pushes.
 */

import { definePlugin, type PluginContext } from "@altnautica/plugin-sdk";

import enLocale from "../../locales/en.json";

/** The event topic the agent half publishes its read-back on. Must equal the
 * manifest skill `state.topic`. */
const STATE_TOPIC = "hello.state";

interface HelloState {
  active?: boolean;
  greeting?: string;
  ticks?: number;
}

definePlugin({
  id: "__PLUGIN_ID__",
  version: "0.1.0",
  locale: enLocale as Record<string, string>,
  async mount(ctx) {
    renderTab(ctx);
  },
});

function renderTab(ctx: PluginContext): void {
  const root = document.body;
  root.style.background = "var(--bg, #0b1220)";
  root.style.color = "var(--fg, #e5e7eb)";
  root.style.font = "13px system-ui, sans-serif";
  root.style.padding = "12px";

  const title = document.createElement("h2");
  title.style.cssText = "margin:0 0 8px;font-size:1rem;font-weight:600";
  title.textContent = ctx.i18n.t("tab.title");

  const hint = document.createElement("p");
  hint.style.cssText = "margin:0 0 12px;color:var(--fg-muted,#94a3b8)";
  hint.textContent = ctx.i18n.t("tab.hint");

  const live = document.createElement("pre");
  live.style.cssText =
    "margin:0;padding:8px;border-radius:6px;background:var(--panel,#0f172a);" +
    "font-family:monospace;white-space:pre-wrap";
  live.dataset.testid = "hello-live";
  live.textContent = ctx.i18n.t("tab.waiting");

  root.replaceChildren(title, hint, live);

  // Subscribe to the agent half's one-way read-back event.
  ctx.events.subscribe<HelloState>(STATE_TOPIC, (state) => {
    live.textContent = formatState(ctx, state);
  });
}

/** Render the agent read-back as a small, honest status block. */
function formatState(ctx: PluginContext, state: HelloState): string {
  const active = state.active === true;
  const lines = [
    `${ctx.i18n.t("live.active")}: ${active ? ctx.i18n.t("live.yes") : ctx.i18n.t("live.no")}`,
    `${ctx.i18n.t("live.greeting")}: ${state.greeting ?? "—"}`,
    `${ctx.i18n.t("live.ticks")}: ${typeof state.ticks === "number" ? state.ticks : 0}`,
  ];
  return lines.join("\n");
}
