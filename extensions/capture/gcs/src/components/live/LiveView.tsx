/**
 * @module LiveView
 * @description A capture device's Live view: REC / MARK with room and floor,
 * the WHEP camera grid, the six health chips, the exposure lock, the
 * return-to-start prompt and the storage left. A Visor also shows its
 * top-down coverage with the walked trajectory; a drone's coverage is a layer
 * on the Map tab, so it gets the one-line summary. A device with no capture
 * service running shows `idle` (a drone: its setup and requirements) or a
 * calm empty state.
 */

import type { ReactNode } from "react";
import { ScanLine } from "lucide-react";

import { CameraTile } from "../../components/capture/CameraTile";
import { CaptureControls } from "../../components/capture/CaptureControls";
import { CoverageMap } from "../../components/capture/CoverageMap";
import { ExposureControl } from "../../components/capture/ExposureControl";
import { HealthChips } from "../../components/capture/HealthChips";
import { ViewerLoading } from "@altnautica/mapping-ui/components/model/viewers/ViewerLoading";
import { Section } from "@altnautica/mapping-ui/components/ui/Section";
import { translator } from "../../host/i18n";
import { useCapture } from "../../hooks/use-capture";
import { cameraEmptyReason } from "../../lib/capture/camera-empty";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";

const t = translator("capture");

export function LiveView({ coverageMap, idle }: { coverageMap: boolean; idle?: ReactNode }) {
  const { client, state, coverage, readiness, loading, absent, stale, refresh } = useCapture();
  if (loading) return <ViewerLoading label={t("loading")} />;
  if (absent || !state) return idle ?? <CalmState icon={ScanLine} message={t("absent")} />;

  return (
    <div className="cap:flex cap:flex-col cap:gap-3 cap:p-3">
      {stale && (
        <p role="status" className="cap:rounded cap:bg-status-warning/15 cap:px-2 cap:py-1 cap:text-[11px] cap:text-status-warning">
          {t("stale")}
        </p>
      )}
      <Section title={t("controls")}>
        <CaptureControls client={client} state={state} coverage={coverage} onChanged={refresh} />
      </Section>
      <div className="cap:grid cap:gap-3 cap:lg:grid-cols-[1fr_auto]">
        <Section title={t("cameras")}>
          {state.frames.perCamera.length === 0 ? (
            <p className="cap:text-[11px] cap:text-text-tertiary">{t(cameraEmptyReason(readiness))}</p>
          ) : (
            <div className="cap:grid cap:gap-2 cap:sm:grid-cols-2">
              {state.frames.perCamera.map((c) => (
                <CameraTile key={c.id} client={client} camera={c} />
              ))}
            </div>
          )}
        </Section>
        <Section title={t("coverage")}>
          {coverageMap && <CoverageMap coverage={coverage} emptyLabel={t("noCoverage")} label={t("coverage")} />}
          <p className="cap:text-[11px] cap:text-text-tertiary">
            {state.coverage.roomPct !== null && t("roomCoverage", { pct: state.coverage.roomPct.toFixed(0) })}
            {state.pose.source && ` · ${t("pose", { source: state.pose.source, health: state.pose.health ?? "–" })}`}
          </p>
          {state.pose.singleFloor && (
            <p role="note" className="cap:text-[11px] cap:text-status-warning">
              {t("hud.planar_single_floor")}
            </p>
          )}
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
