/**
 * @module atlas/WorldModelViewport
 * @description Renders the selected World Model viewer for an artifact. A
 * viewer that throws while rendering shows its error overlay in place instead
 * of taking down the surrounding view. Returns null when there is no artifact
 * (the page shows its empty state).
 */

import { translator } from "../../host/i18n";
import type { ArtifactSource } from "../../lib/net/artifact-source";
import type { ArtifactFrame } from "../../lib/world/placement";
import { ErrorBoundary } from "../ui/ErrorBoundary";
import { SparkViewer } from "../world/SparkViewer";
import { ReconstructionBadge } from "./ReconstructionBadge";
import type { AtlasViewer } from "./viewer-types";
import { PointCloudLodViewer } from "./viewers/PointCloudLodViewer";
import { PointCloudViewer } from "./viewers/PointCloudViewer";
import { RerunViewer } from "./viewers/RerunViewer";
import { ViewerError } from "./viewers/ViewerError";

const t = translator("atlas");

/** The frame every viewer renders into, so the overlays have a box to fill. */
const VIEWER_FRAME = "we:relative we:w-full we:h-full we:min-h-[320px]";

/** The catalogue key naming what failed to load in each viewer's error overlay. */
const VIEWER_WHAT_KEY: Record<AtlasViewer, string> = {
  rerun: "viewerStatus.what.rerun",
  splat: "viewerStatus.what.splat",
  cloud: "viewerStatus.what.pointCloud",
  lod: "viewerStatus.what.pointCloud",
};

function ViewerFor({ viewer, source, frame }: { viewer: AtlasViewer; source: ArtifactSource; frame: ArtifactFrame }) {
  switch (viewer) {
    case "rerun":
      return <RerunViewer source={source} />;
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

export function WorldModelViewport({
  viewer,
  artifact,
  backend = null,
  frame,
}: {
  viewer: AtlasViewer;
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
        label="WorldModelViewport"
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
