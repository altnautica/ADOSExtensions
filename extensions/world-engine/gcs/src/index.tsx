/**
 * World Engine inline GCS module. Mission Control imports this ES module into
 * its own page and calls `mount` once per contributed page; the page rendered
 * is chosen by `host.plugin.panelId` (the manifest's `agent_pages[].id` /
 * `node_surfaces[].id`).
 */

import type { ComponentType } from "react";
import { createRoot } from "react-dom/client";
import type { InlineHostApi, InlinePluginModule } from "@altnautica/plugin-sdk/inline";

import { ToastProvider } from "./components/ui/Toast";
import { HostProvider } from "./host/context";
import { translator } from "./host/i18n";
import { useWorkstationProvisioning } from "./hooks/use-workstation-provisioning";
import { useWorldEngineState } from "./hooks/use-world-engine-state";
import { LiveWorldPage } from "./pages/drone/LiveWorldPage";
import { WorldModelPage } from "./pages/drone/WorldModelPage";
import { WorldModelSetupPage } from "./pages/drone/WorldModelSetupPage";
import { AtlasRelayPage } from "./pages/ground-station/AtlasRelayPage";
import { OffloadPage } from "./pages/offload/OffloadPage";
import { ComputePage } from "./pages/workstation/ComputePage";
import { OverviewPage } from "./pages/workstation/OverviewPage";

/** Every page this module contributes, by manifest id. */
export const PAGES: Record<string, ComponentType> = {
  "world-model": WorldModelPage,
  "world-model-setup": WorldModelSetupPage,
  "live-world": LiveWorldPage,
  offload: OffloadPage,
  "atlas-relay": AtlasRelayPage,
  overview: OverviewPage,
  compute: ComputePage,
};

const t = translator("worldEngine");

/** Per-page wiring every page shares: the node's state feed and the
 * background credential provisioning. */
function PageRoot({ panelId }: { panelId: string }) {
  useWorldEngineState();
  useWorkstationProvisioning();
  const Page = PAGES[panelId];
  return (
    <div className="we:relative we:flex we:min-h-0 we:min-w-0 we:flex-1 we:flex-col">
      {Page ? (
        <Page />
      ) : (
        <p className="we:p-4 we:text-xs we:text-text-tertiary">{t("unknownPage", { id: panelId })}</p>
      )}
    </div>
  );
}

function mountPage(root: HTMLElement, host: InlineHostApi): () => void {
  const reactRoot = createRoot(root);
  reactRoot.render(
    <HostProvider host={host}>
      <ToastProvider>
        <PageRoot panelId={host.plugin.panelId} />
      </ToastProvider>
    </HostProvider>,
  );
  return () => reactRoot.unmount();
}

export const plugin: InlinePluginModule = { mount: mountPage };

export default plugin;
