import { definePlugin } from "@altnautica/plugin-sdk";

definePlugin({
  id: "__PLUGIN_ID__",
  version: "0.1.0",
  async mount(ctx) {
    const root = document.body;
    root.style.background = "var(--bg, #0c0c0c)";
    root.style.color = "var(--fg, #e6e6e6)";
    root.style.font = "12px system-ui, sans-serif";
    root.style.padding = "12px";
    root.textContent = ctx.i18n.t("hello.title");
  },
});
