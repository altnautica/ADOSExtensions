/**
 * @module model/ModelViewport
 * @description Renders the selected model viewer for an artifact. A
 * viewer that throws while rendering shows its error overlay in place instead
 * of taking down the surrounding view. Returns null when there is no artifact
 * (the page shows its empty state).
 */

import { viewerT as t } from "../../host/i18n";
import type { ArtifactSource } from "../../lib/net/artifact-source";
import type { ArtifactFrame } from "../../lib/world/placement";
import { ErrorBoundary } from "../ui/ErrorBoundary";
import { SparkViewer } from "../world/SparkViewer";
import { ReconstructionBadge } from "./ReconstructionBadge";
import type { ModelViewer } from "./viewer-types";
import { PointCloudLodViewer } from "./viewers/PointCloudLodViewer";
import { PointCloudViewer } from "./viewers/PointCloudViewer";
import { useHost } from "../../host/context";
import { RerunViewer } from "./viewers/RerunViewer";
import { ViewerError } from "./viewers/ViewerError";


/** The frame every viewer renders into, so the overlays have a box to fill. */
const VIEWER_FRAME = "map:relative map:w-full map:h-full map:min-h-[320px]";

/** The catalogue key naming what failed to load in each viewer's error overlay. */
const VIEWER_WHAT_KEY: Record<ModelViewer, string> = {
  rerun: "viewerStatus.what.rerun",
  splat: "viewerStatus.what.splat",
  cloud: "viewerStatus.what.pointCloud",
  lod: "viewerStatus.what.pointCloud",
};

function ViewerFor({ viewer, source, frame }: { viewer: ModelViewer; source: ArtifactSource; frame: ArtifactFrame }) {
  const host = useHost();
  switch (viewer) {
    case "rerun":
      return <RerunViewer source={source} readAsset={(path) => host.readAsset(path)} />;
    case "splat":
      return (
        <SparkViewer scene={{ splat: source, placement: frame, collision: null, floorplan: null, rooms: [] }} />
      );
    case "cloud":
      return <PointCloudViewer source={source} />;
    case "lod":
      return <PointCloudLodViewer source={source} />;
  }
}

export function ModelViewport({
  viewer,
  artifact,
  backend = null,
  frame,
}: {
  viewer: ModelViewer;
  artifact: ArtifactSource | null;
  /** The reconstruction backend to name on the viewer, or null. */
  backend?: string | null;
  /** The splat's frame (see `useOutputFrame`): a placement, null for local
   * ENU, or "colmap" for a legacy artifact without a placement. Required so
   * no caller can silently fall into the wrong frame. */
  frame: ArtifactFrame;
}) {
  if (!artifact) return null;
  return (
    <>
      <ErrorBoundary
        label="ModelViewport"
        resetKey={`${viewer}|${artifact.key}`}
        fallback={
          <div className={VIEWER_FRAME}>
            <ViewerError what={t(VIEWER_WHAT_KEY[viewer])} />
          </div>
        }
      >
        {/* Keyed by artifact so a swap starts a fresh stage (no stale bounds). */}
        <ViewerFor key={artifact.key} viewer={viewer} source={artifact} frame={frame} />
      </ErrorBoundary>
      <ReconstructionBadge backend={backend} />
    </>
  );
}
