/**
 * The Capture node surface (drone and Visor): Live, Sessions, and on a drone
 * Model, what the paired node built from this drone's sessions.
 */

import { SegmentedPane, type PaneSegment } from "@altnautica/mapping-ui/components/ui/SegmentedPane";
import { useHost } from "@altnautica/mapping-ui/host/context";

import { DroneLiveView } from "../components/live/DroneLiveView";
import { LiveView } from "../components/live/LiveView";
import { ModelView } from "../components/model/ModelView";
import { SessionsView } from "../components/sessions/SessionsView";
import { translator } from "../host/i18n";
import { useCaptureControl } from "../hooks/use-capture-control";
import { useNodeModel } from "../hooks/use-node-model";

const t = translator("capture");

/** A drone's capture before its service runs: setup and requirements. */
function DroneIdle() {
  const deviceId = useHost().node.deviceId;
  const control = useCaptureControl(deviceId);
  const model = useNodeModel({ sessionId: null });
  return <DroneLiveView control={control} local={model} />;
}

export function CapturePage() {
  const drone = useHost().node.profile === "drone";
  const segments: PaneSegment[] = [
    {
      id: "live",
      label: t("views.live"),
      render: () => <LiveView coverageMap={!drone} idle={drone ? <DroneIdle /> : undefined} />,
    },
    { id: "sessions", label: t("views.sessions"), render: () => <SessionsView /> },
  ];
  if (drone) segments.push({ id: "model", label: t("views.model"), render: () => <ModelView /> });
  return <SegmentedPane ariaLabel={t("views.label")} segments={segments} />;
}
