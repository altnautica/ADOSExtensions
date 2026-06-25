import { describe, it, expect } from "vitest";

import {
  contributesModel,
  createPluginContext,
  MemoryTransport,
  PluginClient,
  PROTOCOL_VERSION,
  type PluginContext,
} from "@altnautica/plugin-sdk";

import enLocale from "../../locales/en.json";

/**
 * The GCS half renders a node-detail tab that subscribes to the agent's
 * `hello.state` event and shows the live read-back. These tests drive the
 * tab through a MemoryTransport-backed context (the same shape the host
 * provides) so the iframe logic is exercised without a real iframe.
 */

const STATE_TOPIC = "hello.state";

function makeCtx(): { ctx: PluginContext; transport: MemoryTransport } {
  const transport = new MemoryTransport();
  const client = new PluginClient({ transport });
  const ctx = createPluginContext({
    client,
    locale: enLocale as Record<string, string>,
  });
  return { ctx, transport };
}

/** Mirror of the bundle's tab renderer for a DOM-free assertion target. */
function renderLive(ctx: PluginContext): HTMLPreElement {
  const live = document.createElement("pre");
  live.textContent = ctx.i18n.t("tab.waiting");
  ctx.events.subscribe<{ active?: boolean; greeting?: string; ticks?: number }>(
    STATE_TOPIC,
    (state) => {
      live.textContent = [
        `${ctx.i18n.t("live.active")}: ${state.active ? ctx.i18n.t("live.yes") : ctx.i18n.t("live.no")}`,
        `${ctx.i18n.t("live.greeting")}: ${state.greeting ?? "—"}`,
        `${ctx.i18n.t("live.ticks")}: ${typeof state.ticks === "number" ? state.ticks : 0}`,
      ].join("\n");
    },
  );
  return live;
}

describe("hello hybrid GCS tab", () => {
  it("resolves the localized title from the bundled locale", () => {
    const { ctx } = makeCtx();
    expect(ctx.i18n.t("tab.title")).toBe("Hello");
    expect(ctx.i18n.t("missing.key")).toBe("missing.key");
  });

  it("renders the agent read-back when a hello.state event arrives", () => {
    const { ctx, transport } = makeCtx();
    const live = renderLive(ctx);
    expect(live.textContent).toBe("Waiting for the agent...");

    transport.pushFromHost({
      id: "evt-1",
      type: "event",
      method: STATE_TOPIC,
      capability: "",
      args: { active: true, greeting: "namaste", ticks: 3 },
      version: PROTOCOL_VERSION,
    });

    expect(live.textContent).toContain("Active: yes");
    expect(live.textContent).toContain("Greeting: namaste");
    expect(live.textContent).toContain("Ticks: 3");
  });
});

describe("manifest contributes model", () => {
  it("types the full contribution surface this template demonstrates", () => {
    const c = contributesModel({
      skills: [
        {
          id: "hello",
          toggle: true,
          activation: { via: "config", config_key: "active" },
          state: { via: "event", topic: STATE_TOPIC },
        },
      ],
      tabs: [{ id: "hello-tab", profile: ["drone"], title: "tab.title" }],
      parameters: [
        {
          key: "rate_hz",
          schema: { type: "number", minimum: 1, maximum: 20, default: 5 },
          ui: { widget: "range" },
        },
        {
          key: "detector",
          binding: "engine.detector",
          schema: { type: "string", default: "coco-person" },
          ui: { widget: "model", task: "detection" },
        },
      ],
      models: [{ id: "coco-person", task: "detection" }],
    });
    expect(c.tabs?.[0]?.profile).toEqual(["drone"]);
    expect(c.parameters?.[1]?.binding).toBe("engine.detector");
  });
});
