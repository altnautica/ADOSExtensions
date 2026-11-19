/**
 * @module pages/offload/enable-options
 * @description The auto | on | off enablement tri-state shared by the drone
 * offload client (`offload.enabled`) and the workstation serving controls
 * (`reconstruct.serving.enabled`).
 */

import type { SelectOption } from "@altnautica/mapping-ui/components/ui/Select";
import { translator } from "../../host/i18n";

export const perceptionT = translator("nodeSettings.perception");

export const ENABLE_OPTIONS: SelectOption[] = [
  { value: "auto", label: perceptionT("enabledAuto") },
  { value: "on", label: perceptionT("enabledOn") },
  { value: "off", label: perceptionT("enabledOff") },
];
