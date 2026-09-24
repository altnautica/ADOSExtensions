/**
 * @module atlas/WorldGenerationCard
 * @description The shared-data world model for one drone: what the paired
 * compute node's newest reconstruct generation actually produced, read off the
 * per-device descriptor stream (`GET /ws/atlas/<deviceId>`).
 *
 * This is the consumer side of the four `plugin.atlas.*` artifact topics. It is
 * NOT the viewer — the viewer renders an artifact URL, while this states what
 * the generation contains, which is the part an operator plans against.
 *
 * Strings live in the existing top-level `atlas` namespace under a `world*`
 * prefix, beside `worldModelHeading` / `worldModelEmpty` / `liveWorld*`, and
 * three already-present keys are reused rather than duplicated:
 * `worldModelHeading` for the card title, `worldModelEmpty` as the absent-state
 * body, and `worldModelNoNode` for the no-compute-node stream status (the same
 * meaning it already carries in `DroneLiveWorldTab`).
 *
 * Two honesty properties carried through from the producer are the reason this
 * component exists rather than a row of numbers:
 *
 *  - **"No world model" is not "an empty world".** The node publishes NOTHING
 *    for a generation with nothing readable, so silence means no world model was
 *    produced. Rendering that as an empty world would tell an operator the
 *    volume is surveyed and clear when nothing has been surveyed at all, so the
 *    two get different copy, and a third state covers "a generation arrived but
 *    stated no readable count". The load-bearing half of the absent state is
 *    `worldAbsentTitle`, a string this surface owns; the reused
 *    `worldModelEmpty` body only elaborates it, so a later edit to that
 *    misleadingly-named key cannot silently turn absent into empty.
 *  - **A missing count is unknown, never zero.** See `WorldArtifactBlock`.
 *
 */

import { LOCALE, translator } from "../../host/i18n";
import { Boxes } from "lucide-react";

import {
  TextRow,
  WorldArtifactBlock,
  type WorldTranslate,
} from "./WorldArtifactBlock";
import {
  worldModelPresence,
  type WorldModelGeneration,
} from "../../lib/atlas/world-generation";
import { useAtlasWorldStream } from "../../hooks/use-atlas-world-stream";
import {
  selectDeviceWorld,
  useAtlasWorldStore,
  type DeviceWorldState,
  type WorldStreamStatus,
} from "../../stores/atlas-world-store";

const t = translator("atlas");

/** i18n key for each stream stand-down / connection cause. Every value is a
 * distinct operator-facing reason, and none of them means "no world model" —
 * that is a statement about the data, not the transport. `no-node` reuses the
 * existing `worldModelNoNode`, which already says exactly this. */
const STREAM_STATUS_KEY: Record<WorldStreamStatus, string> = {
  idle: "worldStreamIdle",
  "no-node": "worldModelNoNode",
  "blocked-origin": "worldStreamBlockedOrigin",
  connecting: "worldStreamConnecting",
  connected: "worldStreamConnected",
  reconnecting: "worldStreamReconnecting",
};

/** The generation's artifact grid, plus the explanation when the generation
 * carries no content (or none that was stated). */
function GenerationBody({
  generation,
  t,
  locale,
}: {
  generation: WorldModelGeneration | null;
  t: WorldTranslate;
  locale: string;
}) {
  const presence = worldModelPresence(generation);
  if (generation === null) {
    return (
      <div data-testid="world-presence" data-presence={presence}>
        <div className="we:text-xs we:font-medium we:text-text-primary">
          {t("worldAbsentTitle")}
        </div>
        {/* Reused: "No reconstruction yet. A captured session reconstructs on
            the compute node, then appears here." — honest for this state, and
            it was dead in the catalogue. */}
        <p className="we:mt-1 we:text-[11px] we:text-text-tertiary">
          {t("worldModelEmpty")}
        </p>
      </div>
    );
  }
  return (
    <div
      className="we:space-y-2"
      data-testid="world-presence"
      data-presence={presence}
    >
      {presence === "empty" && (
        <div>
          <div className="we:text-xs we:font-medium we:text-text-primary">
            {t("worldEmptyTitle")}
          </div>
          <p className="we:mt-1 we:text-[11px] we:text-text-tertiary">
            {t("worldEmptyBody")}
          </p>
        </div>
      )}
      {presence === "unknown" && (
        <div>
          <div className="we:text-xs we:font-medium we:text-text-primary">
            {t("worldUnknownTitle")}
          </div>
          <p className="we:mt-1 we:text-[11px] we:text-text-tertiary">
            {t("worldUnknownBody")}
          </p>
        </div>
      )}
      <div className="we:grid we:grid-cols-1 we:sm:grid-cols-2 we:gap-2">
        <WorldArtifactBlock
          kind="splat"
          artifact={generation.splat}
          t={t}
          locale={locale}
        />
        <WorldArtifactBlock
          kind="pointcloud"
          artifact={generation.pointcloud}
          t={t}
          locale={locale}
        />
        <WorldArtifactBlock
          kind="mesh"
          artifact={generation.mesh}
          t={t}
          locale={locale}
        />
        <WorldArtifactBlock
          kind="occupancy"
          artifact={generation.occupancy}
          t={t}
          locale={locale}
        />
      </div>
    </div>
  );
}

/** Refusals and drops the stream accounted for. Rendered only when non-zero: a
 * silent skip is what makes a sparse world model look complete. */
function StreamAccounting({
  world,
  t,
  locale,
}: {
  world: DeviceWorldState;
  t: WorldTranslate;
  locale: string;
}) {
  const notes: string[] = [];
  if (world.supersededDescriptors > 0) {
    notes.push(
      t("worldSuperseded", {
        count: world.supersededDescriptors.toLocaleString(locale),
      }),
    );
  }
  if (world.versionRejectedFrames > 0) {
    notes.push(
      t("worldVersionRejected", {
        count: world.versionRejectedFrames.toLocaleString(locale),
        version: world.rejectedVersion ?? "?",
      }),
    );
  }
  const badFrames = world.malformedFrames + world.shapeRejectedFrames;
  if (badFrames > 0) {
    notes.push(t("worldMalformed", { count: badFrames.toLocaleString(locale) }));
  }
  if (notes.length === 0) return null;
  return (
    <ul
      className="we:text-[10px] we:text-status-warning we:space-y-0.5"
      data-testid="world-stream-accounting"
    >
      {notes.map((n) => (
        <li key={n}>{n}</li>
      ))}
    </ul>
  );
}

/**
 * The world-model generation card for one drone.
 *
 * @param droneDeviceId The capturing drone's device id — the descriptor stream
 *   path and the store key, so a node serving several drones never cross-talks.
 * @param computeNodeDeviceId The reconstructor node's device id, from
 *   `useDroneWorldModel().computeNodeDeviceId`.
 */
export function WorldGenerationCard({
  droneDeviceId,
  computeNodeDeviceId,
}: {
  droneDeviceId: string | null | undefined;
  computeNodeDeviceId: string | null | undefined;
}) {
  useAtlasWorldStream(droneDeviceId, computeNodeDeviceId);
  const world = useAtlasWorldStore(selectDeviceWorld(droneDeviceId));
  const { generation } = world;

  return (
    <section
      className="we:border we:border-border-default we:rounded-lg we:p-3 we:space-y-2"
      aria-label={t("worldModelHeading")}
    >
      <div className="we:flex we:items-center we:gap-1.5">
        <Boxes className="we:w-3.5 we:h-3.5 we:text-text-tertiary" />
        <span className="we:text-xs we:font-medium we:text-text-secondary">
          {t("worldModelHeading")}
        </span>
        {generation !== null && (
          <span
            className="we:ml-auto we:font-mono we:text-[10px] we:text-text-tertiary we:tabular-nums"
            data-testid="world-generation"
          >
            {t("worldGeneration", { generation: generation.generation })}
          </span>
        )}
      </div>

      {generation !== null && generation.sessionId.length > 0 && (
        <TextRow label={t("worldSession")} value={generation.sessionId} />
      )}

      <GenerationBody generation={generation} t={t} locale={LOCALE} />

      <div
        className="we:text-[10px] we:text-text-tertiary"
        data-testid="world-stream-status"
      >
        {t(STREAM_STATUS_KEY[world.status])}
      </div>
      <StreamAccounting world={world} t={t} locale={LOCALE} />
    </section>
  );
}
