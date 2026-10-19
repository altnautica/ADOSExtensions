/**
 * @module CapturePage
 * @description A capture device's live capture page (Visor node surface
 * `capture`): the WHEP camera grid, top-down coverage with the walked
 * trajectory, the six health chips, REC / MARK with room and floor, the
 * exposure lock, the return-to-start prompt and the storage left. A device
 * with no capture service shows a calm empty state.
 */

import { ScanLine } from "lucide-react";

import { CameraTile } from "../../components/capture/CameraTile";
import { CaptureControls } from "../../components/capture/CaptureControls";
import { CoverageMap } from "../../components/capture/CoverageMap";
import { ExposureControl } from "../../components/capture/ExposureControl";
import { HealthChips } from "../../components/capture/HealthChips";
import { ViewerLoading } from "../../components/atlas/viewers/ViewerLoading";
import { Section } from "../../components/ui/Section";
import { translator } from "../../host/i18n";
import { useCapture } from "../../hooks/use-capture";
import { CalmState } from "../workstation/CalmState";

const t = translator("capture");

export function CapturePage() {
  const { client, state, coverage, loading, absent, stale, refresh } = useCapture();
  if (loading) return <ViewerLoading label={t("loading")} />;
  if (absent || !state) return <CalmState icon={ScanLine} message={t("absent")} />;

  return (
    <div className="we:flex we:flex-col we:gap-3 we:p-3">
      {stale && (
        <p role="status" className="we:rounded we:bg-status-warning/15 we:px-2 we:py-1 we:text-[11px] we:text-status-warning">
          {t("stale")}
        </p>
      )}
      <Section title={t("controls")}>
        <CaptureControls client={client} state={state} coverage={coverage} onChanged={refresh} />
      </Section>
      <div className="we:grid we:gap-3 we:lg:grid-cols-[1fr_auto]">
        <Section title={t("cameras")}>
          {state.frames.perCamera.length === 0 ? (
            <p className="we:text-[11px] we:text-text-tertiary">{t("noCameras")}</p>
          ) : (
            <div className="we:grid we:gap-2 we:sm:grid-cols-2">
              {state.frames.perCamera.map((c) => (
                <CameraTile key={c.id} client={client} camera={c} />
              ))}
            </div>
          )}
        </Section>
        <Section title={t("coverage")}>
          <CoverageMap coverage={coverage} emptyLabel={t("noCoverage")} label={t("coverage")} />
          <p className="we:text-[11px] we:text-text-tertiary">
            {state.coverage.roomPct !== null && t("roomCoverage", { pct: state.coverage.roomPct.toFixed(0) })}
            {state.pose.source && ` · ${t("pose", { source: state.pose.source, health: state.pose.health ?? "–" })}`}
          </p>
        </Section>
      </div>
      <Section title={t("healthTitle")}>
        <HealthChips signals={state.health} />
      </Section>
      <Section title={t("exposure")}>
        <ExposureControl client={client} exposure={state.exposure} onChanged={refresh} />
      </Section>
    </div>
  );
}
