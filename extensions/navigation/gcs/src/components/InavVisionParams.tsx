import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { tr } from "../i18n";

import { ParamCard, subheading, type ParamGroup } from "./ParamCard";

const GROUPS: ParamGroup[] = [
  { id: "flow", rows: ["opflow_hardware", "opflow_scale"] },
  {
    id: "nav",
    rows: ["nav_use_optflow_for_poshold", "nav_extra_arming_safety", "nav_landing_speed"],
  },
  { id: "rangefinder", rows: ["rangefinder_hardware", "nav_rangefinder_for_terrain"] },
];

const NOTES = ["version", "serial", "component"];

export function InavVisionParams({ ctx }: { ctx: PluginContext }): JSX.Element {
  const t = ctx.i18n.t;
  return (
    <ParamCard t={t} prefix="navigation.params.inav" groups={GROUPS} testId="vn-inav-params">
      <div>
        <p style={subheading}>{tr(t, "navigation.params.inav.notesTitle")}</p>
        <ul style={noteList}>
          {NOTES.map((id) => (
            <li key={id}>{tr(t, `navigation.params.inav.notes.${id}`)}</li>
          ))}
        </ul>
      </div>
    </ParamCard>
  );
}

const noteList: CSSProperties = {
  fontSize: "0.75rem",
  color: "var(--vn-text-3, #94a3b8)",
  margin: 0,
  paddingLeft: "1.25rem",
};
