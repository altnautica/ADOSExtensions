/**
 * Capture inline GCS module. Mission Control imports this ES module into its
 * own page and calls `mount` once per contributed page or panel; the one
 * rendered is chosen by `host.plugin.panelId` (the manifest's
 * `node_surfaces[].id`, `panels[].id` or `agent_pages[].id`).
 */

import type { ComponentType } from "react";
import { createRoot } from "react-dom/client";
import type { InlineHostApi, InlinePluginModule } from "@altnautica/plugin-sdk/inline";

import { ToastProvider } from "@altnautica/mapping-ui/components/ui/Toast";
import { useCredentialProvisioning } from "@altnautica/mapping-ui/hooks/use-credential-provisioning";
import { HostProvider } from "@altnautica/mapping-ui/host/context";

import { translator } from "./host/i18n";
import { useCaptureState } from "./hooks/use-capture-state";
import { CAPTURE_CREDENTIALS } from "@altnautica/mapping-ui/lib/nodes/credential-targets";
import { CalibrationPage } from "./pages/CalibrationPage";
import { CapturePage } from "./pages/CapturePage";
import { CaptureSetupPage } from "./pages/CaptureSetupPage";
import { RelayPage } from "./pages/RelayPage";
import { CaptureChips } from "./panels/CaptureChips";
import { CoverageOverlay } from "./panels/CoverageOverlay";

/** Every page and panel this module contributes, by manifest id. */
export const PAGES: Record<string, ComponentType> = {
  capture: CapturePage,
  "capture-chips": CaptureChips,
  "capture-coverage": CoverageOverlay,
  "capture-setup": CaptureSetupPage,
  calibration: CalibrationPage,
  relay: RelayPage,
};

const t = translator("capture");

/** Per-page wiring every page shares: the node's state feed and the
 * background credential provisioning. */
function PageRoot({ panelId }: { panelId: string }) {
  useCaptureState();
  useCredentialProvisioning(CAPTURE_CREDENTIALS);
  const Page = PAGES[panelId];
  return (
    <div className="cap:relative cap:flex cap:min-h-0 cap:min-w-0 cap:flex-1 cap:flex-col">
      {Page ? (
        <Page />
      ) : (
        <p className="cap:p-4 cap:text-xs cap:text-text-tertiary">{t("unknownPage", { id: panelId })}</p>
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
