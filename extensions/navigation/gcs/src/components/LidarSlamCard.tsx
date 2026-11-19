import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { noDataLabel, type Freshness } from "../freshness";
import { DASH, tr } from "../i18n";
import type { LidarStatus, MapStatus } from "../lidarSlam";

interface Props {
  ctx: PluginContext;
  map: MapStatus;
  /** How current the map status is; `null` before the first one. */
  mapFresh: Freshness;
  lidar: LidarStatus;
  lidarFresh: Freshness;
  /** Whether a LiDAR status ever arrived: none when the rig declares none. */
  lidarSeen: boolean;
}

interface Row {
  id: string;
  key: string;
  text: string;
}

/**
 * The lidar-slam service in three blocks: the pose provider it runs, the
 * configured LiDAR's link, and the live map's memory, placement and area.
 * The map itself is drawn on the node's Map tab.
 */
export function LidarSlamCard({ ctx, map, mapFresh, lidar, lidarFresh, lidarSeen }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const mapLive = mapFresh.kind === "live";
  const provider: Row[] = [
    {
      id: "provider",
      key: "provider",
      text: !mapLive
        ? noDataLabel(t, mapFresh)
        : map.poseProvider
          ? tr(t, `navigation.lidarSlam.providers.${map.poseProvider}`)
          : tr(t, "navigation.lidarSlam.providers.none"),
    },
  ];
  const link: Row[] = !lidarSeen
    ? []
    : [
        {
          id: "lidar-link",
          key: "link",
          text:
            lidarFresh.kind !== "live"
              ? noDataLabel(t, lidarFresh)
              : lidar.connected
                ? tr(t, "navigation.lidarSlam.connected")
                : tr(t, "navigation.lidarSlam.disconnected"),
        },
        {
          id: "lidar-rate",
          key: "rate",
          text: lidar.rateHz === null ? DASH : tr(t, "navigation.format.hz", { v: lidar.rateHz.toFixed(1) }),
        },
        {
          id: "lidar-returns",
          key: "returns",
          text: lidar.returnsLastScan === null ? DASH : `${lidar.returnsLastScan}`,
        },
        { id: "lidar-device", key: "device", text: lidar.device ?? DASH },
      ];
  const memory =
    map.memoryUsedMb === null || map.memoryBudgetMb === null
      ? DASH
      : tr(t, "navigation.lidarSlam.memoryOf", {
          used: Math.round(map.memoryUsedMb),
          budget: Math.round(map.memoryBudgetMb),
        });
  const live: Row[] = [
    { id: "map-memory", key: "memory", text: mapLive ? memory : DASH },
    {
      id: "map-placement",
      key: "placement",
      text: mapLive && map.placement ? tr(t, `navigation.lidarSlam.placements.${map.placement}`) : DASH,
    },
    {
      id: "map-area",
      key: "area",
      text: mapLive && map.areaM2 !== null ? tr(t, "navigation.lidarSlam.areaM2", { v: Math.round(map.areaM2) }) : DASH,
    },
  ];
  const rows = (list: Row[]) =>
    list.map((r) => (
      <div style={row} key={r.id}>
        <span style={label}>{tr(t, `navigation.lidarSlam.${r.key}`)}</span>
        <span style={value} data-testid={`vn-slam-${r.id}`}>
          {r.text}
        </span>
      </div>
    ));

  return (
    <section style={mapFresh.kind === "stale" ? { ...card, opacity: 0.5 } : card} data-testid="vn-lidar-slam">
      <h3 style={heading}>{tr(t, "navigation.lidarSlam.poseTitle")}</h3>
      {rows(provider)}
      <h3 style={heading}>{tr(t, "navigation.lidarSlam.lidarTitle")}</h3>
      {lidarSeen ? (
        rows(link)
      ) : (
        <span style={label} data-testid="vn-slam-no-lidar">
          {tr(t, "navigation.lidarSlam.noLidar")}
        </span>
      )}
      <h3 style={heading}>{tr(t, "navigation.lidarSlam.mapTitle")}</h3>
      {rows(live)}
      <span style={label}>{tr(t, "navigation.lidarSlam.mapTab")}</span>
    </section>
  );
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  display: "flex",
  flexDirection: "column",
  gap: "0.5rem",
  color: "var(--vn-text, #ededed)",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  color: "var(--vn-text-2, #b4b4b4)",
  margin: 0,
};
const row: CSSProperties = {
  display: "flex",
  justifyContent: "space-between",
  alignItems: "baseline",
  fontSize: "0.8125rem",
};
const label: CSSProperties = { color: "var(--vn-text-3, #94a3b8)" };
const value: CSSProperties = {
  fontVariantNumeric: "tabular-nums",
  color: "var(--vn-text, #ededed)",
};
