/**
 * @module atlas/WorldModelViewport
 * @description Renders the selected World Model viewer for an artifact. A
 * viewer that throws while rendering shows its error overlay in place instead
 * of taking down the surrounding view. Returns null when there is no artifact
 * (the page shows its empty state).
 */

import type { ArtifactSource } from "../../lib/net/artifact-source";
import { ErrorBoundary } from "../ui/ErrorBoundary";
import { SparkViewer } from "../world/SparkViewer";
import { ReconstructionBadge } from "./ReconstructionBadge";
import type { AtlasViewer } from "./viewer-types";
import { PointCloudLodViewer } from "./viewers/PointCloudLodViewer";
import { PointCloudViewer } from "./viewers/PointCloudViewer";
import { RerunViewer } from "./viewers/RerunViewer";
import { ViewerError } from "./viewers/ViewerError";

/** The frame every viewer renders into, so the overlays have a box to fill. */
const VIEWER_FRAME = "we:relative we:w-full we:h-full we:min-h-[320px]";

/** The name each viewer's error overlay uses for what failed to load. */
const VIEWER_WHAT: Record<AtlasViewer, string> = {
  rerun: "Rerun",
  splat: "splat",
  cloud: "point cloud",
  lod: "point cloud",
};

function ViewerFor({ viewer, source }: { viewer: AtlasViewer; source: ArtifactSource }) {
  switch (viewer) {
    case "rerun":
      return <RerunViewer source={source} />;
    case "splat":
      return (
        <SparkViewer scene={{ splat: source, placement: null, collision: null, floorplan: null, rooms: [] }} />
      );
    case "cloud":
      return <PointCloudViewer source={source} />;
    case "lod":
      return <PointCloudLodViewer source={source} />;
  }
}

export function WorldModelViewport({
  viewer,
  artifact,
  backend = null,
}: {
  viewer: AtlasViewer;
  artifact: ArtifactSource | null;
  /** The reconstruction backend to name on the viewer, or null. */
  backend?: string | null;
}) {
  if (!artifact) return null;
  return (
    <>
      <ErrorBoundary
        label="WorldModelViewport"
        resetKey={`${viewer}|${artifact.key}`}
        fallback={
          <div className={VIEWER_FRAME}>
            <ViewerError what={VIEWER_WHAT[viewer]} />
          </div>
        }
      >
        <ViewerFor viewer={viewer} source={artifact} />
      </ErrorBoundary>
      <ReconstructionBadge backend={backend} />
    </>
  );
}
