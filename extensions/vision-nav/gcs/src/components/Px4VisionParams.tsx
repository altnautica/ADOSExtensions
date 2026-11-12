import type { PluginContext } from "@altnautica/plugin-sdk";

import { ParamCard, type ParamGroup } from "./ParamCard";

/** Shown only when the flight controller reports them: the PX4 1.14+
 * flow control and the older aid mask are alternatives. */
const OPTIONAL: Record<string, true> = { EKF2_OF_CTRL: true, EKF2_AID_MASK: true };

const GROUPS: ParamGroup[] = [
  { id: "flow", rows: ["EKF2_OF_CTRL", "SENS_FLOW_MAXR"] },
  { id: "aid", rows: ["EKF2_AID_MASK", "EKF2_HGT_REF"] },
];

export interface Px4VisionParamsProps {
  ctx: PluginContext;
  /** Watched parameters the flight controller has reported
   * (``fcParams`` on the navigation telemetry). */
  fcParams?: readonly string[];
}

export function Px4VisionParams({ ctx, fcParams = [] }: Px4VisionParamsProps): JSX.Element {
  const reported = new Set(fcParams);
  const groups = GROUPS.map((group) => ({
    ...group,
    rows: group.rows.filter((id) => !OPTIONAL[id] || reported.has(id)),
  })).filter((group) => group.rows.length > 0);
  return (
    <ParamCard t={ctx.i18n.t} prefix="navigation.params.px4" groups={groups} testId="vn-px4-params" />
  );
}
