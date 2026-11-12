import type { PluginContext } from "@altnautica/plugin-sdk";

import { ParamCard, type ParamGroup } from "./ParamCard";

/**
 * Every ArduPilot parameter `required_fc_parameters.ardupilot` in the
 * manifest names, grouped by the mode that needs it, plus the source-set
 * and flow-tuning parameters the optical-flow modes read.
 */
const GROUPS: ParamGroup[] = [
  { id: "drivers", rows: ["FLOW_TYPE"] },
  {
    id: "sourceSets",
    rows: [
      "EK3_SRC1_POSXY",
      "EK3_SRC1_VELXY",
      "EK3_SRC1_YAW",
      "EK3_SRC2_POSXY",
      "EK3_SRC2_VELXY",
      "EK3_SRC3_POSXY",
      "EK3_SRC3_VELXY",
      "EK3_SRC_OPTIONS",
    ],
  },
  {
    id: "flowTuning",
    rows: ["EK3_FLOW_DELAY", "EK3_FLOW_QUAL_MIN", "FLOW_HEIGHT_MIN", "FLOW_HEIGHT_MAX"],
  },
  {
    id: "externalNav",
    rows: [
      "VISO_TYPE",
      "VISO_DELAY_MS",
      "AHRS_OPTIONS",
      "AHRS_ORIGIN_LAT",
      "AHRS_ORIGIN_LON",
      "AHRS_ORIGIN_ALT",
      "COMPASS_AUTODEC",
      "EK3_MAG_EF_LIM",
      "ARMING_SKIPCHK",
    ],
  },
  { id: "obstacle", rows: ["PRX1_TYPE"] },
];

export function ArduPilotVisionParams({ ctx }: { ctx: PluginContext }): JSX.Element {
  return (
    <ParamCard
      t={ctx.i18n.t}
      prefix="navigation.params.ardupilot"
      groups={GROUPS}
      testId="vn-ardupilot-params"
    />
  );
}
