/**
 * Reconstruct inline GCS module. Mission Control imports this ES module into
 * its own page and calls `mount` once per contributed page; the page rendered
 * is chosen by `host.plugin.panelId` (the manifest's `node_surfaces[].id` /
 * `agent_pages[].id`).
 */

import type { ComponentType } from "react";
import { createRoot } from "react-dom/client";
import type { InlineHostApi, InlinePluginModule } from "@altnautica/plugin-sdk/inline";

import { ToastProvider } from "@altnautica/mapping-ui/components/ui/Toast";
import { useCredentialProvisioning } from "@altnautica/mapping-ui/hooks/use-credential-provisioning";
import { HostProvider } from "@altnautica/mapping-ui/host/context";
import { RECONSTRUCT_CREDENTIALS } from "@altnautica/mapping-ui/lib/nodes/credential-targets";

import { translator } from "./host/i18n";
import { useReconstructState } from "./hooks/use-reconstruct-state";
import { OffloadPage } from "./pages/offload/OffloadPage";
import { ReconstructPage } from "./pages/ReconstructPage";

/** Every page this module contributes, by manifest id. */
export const PAGES: Record<string, ComponentType> = {
  reconstruct: ReconstructPage,
  offload: OffloadPage,
};

const t = translator("reconstruct");

/** Per-page wiring every page shares: the node's state feed and the
 * background credential provisioning. */
function PageRoot({ panelId }: { panelId: string }) {
  useReconstructState();
  useCredentialProvisioning(RECONSTRUCT_CREDENTIALS);
  const Page = PAGES[panelId];
  return (
    <div className="rec:relative rec:flex rec:min-h-0 rec:min-w-0 rec:flex-1 rec:flex-col">
      {Page ? (
        <Page />
      ) : (
        <p className="rec:p-4 rec:text-xs rec:text-text-tertiary">{t("unknownPage", { id: panelId })}</p>
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
