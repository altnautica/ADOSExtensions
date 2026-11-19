import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { tr } from "../i18n";

const LINES = ["estimator", "ekf", "upstream", "crossFlash"];

export function BetaflightUnsupported({ ctx }: { ctx: PluginContext }): JSX.Element {
  const t = ctx.i18n.t;
  return (
    <section style={card} data-testid="vn-betaflight-unsupported">
      <h3 style={heading}>{tr(t, "navigation.params.betaflight.title")}</h3>
      <ul style={list}>
        {LINES.map((id) => (
          <li key={id}>{tr(t, `navigation.params.betaflight.lines.${id}`)}</li>
        ))}
      </ul>
    </section>
  );
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text-2, #b4b4b4)",
  display: "flex",
  flexDirection: "column",
  gap: "0.5rem",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  margin: 0,
  color: "var(--vn-text, #ededed)",
};
const list: CSSProperties = {
  fontSize: "0.8125rem",
  color: "var(--vn-text-3, #94a3b8)",
  margin: 0,
  paddingLeft: "1.25rem",
};
