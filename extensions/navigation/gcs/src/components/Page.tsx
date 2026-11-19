import type { PluginContext } from "@altnautica/plugin-sdk";

import { tr } from "../i18n";
import type { FirmwareType } from "../types";

import { NavigationHealthPanel } from "./NavigationHealthPanel";
import { NavigationTab } from "./NavigationTab";

/** The contributions this bundle renders, by panel id. */
export const HEALTH_PANEL_ID = "navigation-health";
export const AGENT_PAGE_ID = "navigation";

/** The tree for the contribution the frame was mounted as. */
export function Page({ ctx, firmware }: { ctx: PluginContext; firmware: FirmwareType }): JSX.Element {
  switch (ctx.mount.panelId) {
    case HEALTH_PANEL_ID:
      return <NavigationHealthPanel ctx={ctx} />;
    case AGENT_PAGE_ID:
      return <NavigationTab ctx={ctx} firmware={firmware} />;
    default:
      return (
        <p role="status" data-testid="vn-unknown-page">
          {tr(ctx.i18n.t, "navigation.unknownPage", { id: ctx.mount.panelId ?? "" })}
        </p>
      );
  }
}
