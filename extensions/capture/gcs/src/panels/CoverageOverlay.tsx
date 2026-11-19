/**
 * The map's Coverage layer: the open session's covered cells and walked
 * trajectory, drawn in its own session frame as an inset over the map (the
 * overlay slot hands a module the map's box, not its projection, so the
 * cells are not geo-registered on the tiles).
 */

import { CoverageMap } from "../components/capture/CoverageMap";
import { translator } from "../host/i18n";
import { useCapture } from "../hooks/use-capture";

const t = translator("capture");

export function CoverageOverlay() {
  const { coverage, state } = useCapture();
  if (!state || state.state === "idle") return null;
  return (
    <div className="cap:pointer-events-none cap:absolute cap:right-3 cap:bottom-12 cap:w-56 cap:rounded-lg cap:border cap:border-border-default cap:bg-bg-primary/85 cap:p-2">
      <CoverageMap coverage={coverage} emptyLabel={t("noCoverage")} label={t("coverage")} />
    </div>
  );
}
