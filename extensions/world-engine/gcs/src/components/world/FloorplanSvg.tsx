/**
 * @module world/FloorplanSvg
 * @description One floor of a reconstruction's floorplan as a north-up SVG:
 * walls, door gaps, room outlines with name and area, an optional "you are
 * here" marker, and click-to-jump on a room.
 */

import { useMemo } from "react";

import {
  floorBounds,
  planTransform,
  roomLabelPoint,
  type FloorplanFloor,
  type FloorRoom,
} from "../../lib/world/floorplan";
import { formatArea } from "../../lib/world/measure";

export function FloorplanSvg({
  floor,
  width,
  height,
  here,
  onRoom,
}: {
  floor: FloorplanFloor;
  width: number;
  height: number;
  /** Viewer position in plan metres `[east, north]` and heading (rad). */
  here?: { x: number; y: number; heading: number } | null;
  onRoom?: (room: FloorRoom) => void;
}) {
  const tf = useMemo(() => {
    const b = floorBounds(floor);
    return b ? planTransform(b, width, height) : null;
  }, [floor, width, height]);
  if (!tf) return null;
  const line = ([x0, y0, x1, y1]: [number, number, number, number]) => {
    const [a, b] = tf.toPx(x0, y0);
    const [c, d] = tf.toPx(x1, y1);
    return { x1: a, y1: b, x2: c, y2: d };
  };
  const herePx = here ? tf.toPx(here.x, here.y) : null;

  return (
    <svg width={width} height={height} viewBox={`0 0 ${width} ${height}`} className="we:block">
      {floor.rooms.map((r, i) => {
        const pts = r.polygon.map(([x, y]) => tf.toPx(x, y).join(",")).join(" ");
        const label = roomLabelPoint(r);
        const lp = label ? tf.toPx(label[0], label[1]) : null;
        return (
          <g
            key={`${r.name}-${i}`}
            className={onRoom ? "we:cursor-pointer" : undefined}
            onClick={onRoom ? () => onRoom(r) : undefined}
          >
            {pts && <polygon points={pts} fill="currentColor" fillOpacity={0.06} stroke="none" />}
            {lp && (
              <text x={lp[0]} y={lp[1]} textAnchor="middle" fontSize={10} fill="currentColor">
                <tspan x={lp[0]}>{r.name}</tspan>
                {r.areaM2 !== null && (
                  <tspan x={lp[0]} dy={12} fillOpacity={0.6} fontSize={9}>
                    {formatArea(r.areaM2)}
                  </tspan>
                )}
              </text>
            )}
          </g>
        );
      })}
      {floor.walls.map((w, i) => (
        <line key={`w${i}`} {...line(w)} stroke="currentColor" strokeWidth={2} strokeLinecap="round" />
      ))}
      {floor.doors.map((d, i) => (
        <line key={`d${i}`} {...line(d)} stroke="#38bdf8" strokeWidth={2} strokeDasharray="3 3" />
      ))}
      {herePx && here && (
        <g transform={`translate(${herePx[0]} ${herePx[1]}) rotate(${(-here.heading * 180) / Math.PI})`}>
          <path d="M0,-7 L5,5 L0,2 L-5,5 Z" fill="#facc15" />
        </g>
      )}
    </svg>
  );
}
