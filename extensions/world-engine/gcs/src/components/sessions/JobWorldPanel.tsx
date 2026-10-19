/**
 * @module sessions/JobWorldPanel
 * @description Opens one finished reconstruction: the Spark world viewer
 * (splat, floorplan, hotspots, measure on the node's mesh), the LAZ point
 * cloud, and the tour bundle download.
 */

import { useState } from "react";
import { Boxes, Download } from "lucide-react";

import { PointCloudViewer } from "../atlas/viewers/PointCloudViewer";
import { ViewerLoading } from "../atlas/viewers/ViewerLoading";
import { Button } from "../ui/Button";
import { useToast } from "../ui/Toast";
import { SparkViewer } from "../world/SparkViewer";
import { translator } from "../../host/i18n";
import { useJobWorld } from "../../hooks/use-job-world";
import type { ComputeAgentClient } from "../../lib/agent/compute-client";
import type { ArtifactSource } from "../../lib/net/artifact-source";
import { CalmState } from "../../pages/workstation/CalmState";

const t = translator("sessions");

/** Save an artifact through the browser's download. */
export async function downloadArtifact(source: ArtifactSource, fileName: string): Promise<boolean> {
  try {
    const res = await source.fetch(AbortSignal.timeout(10 * 60_000));
    if (!res.ok) return false;
    const url = URL.createObjectURL(await res.blob());
    const a = document.createElement("a");
    a.href = url;
    a.download = fileName;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 30_000);
    return true;
  } catch {
    return false;
  }
}

export function JobWorldPanel({ client, jobId }: { client: ComputeAgentClient; jobId: string }) {
  const world = useJobWorld(client, jobId);
  const { toast } = useToast();
  const [picked, setView] = useState<"splat" | "cloud" | null>(null);
  const [saving, setSaving] = useState(false);

  if (world.status === "loading" || world.status === "idle") return <ViewerLoading label={t("openingWorld")} />;
  if (world.status === "failed") return <CalmState icon={Boxes} message={t("worldUnavailable")} />;
  const { scene, pointcloud, tour } = world.world;
  const view = picked ?? (scene.splat ? "splat" : "cloud");

  return (
    <div className="we:flex we:flex-col we:gap-2">
      <div className="we:flex we:items-center we:gap-1">
        {(["splat", "cloud"] as const).map((v) => (
          <Button
            key={v}
            size="sm"
            variant={view === v ? "primary" : "ghost"}
            disabled={v === "cloud" && !pointcloud}
            onClick={() => setView(v)}
          >
            {t(`view.${v}`)}
          </Button>
        ))}
        {tour && (
          <Button
            size="sm"
            variant="secondary"
            className="we:ml-auto"
            loading={saving}
            icon={<Download size={12} />}
            onClick={async () => {
              setSaving(true);
              const ok = await downloadArtifact(tour, `tour-${jobId}.zip`);
              setSaving(false);
              if (!ok) toast(t("downloadFailed"), "error");
            }}
          >
            {t("downloadTour")}
          </Button>
        )}
      </div>
      <div className="we:relative we:h-[460px]">
        {view === "cloud" && pointcloud ? (
          <PointCloudViewer source={pointcloud} />
        ) : scene.splat ? (
          <SparkViewer
            scene={scene}
            measureRemote={async (a, b) => (await client.measure(jobId, a, b))?.distanceM ?? null}
          />
        ) : (
          <CalmState icon={Boxes} message={t("noSplat")} />
        )}
      </div>
    </div>
  );
}
