/**
 * @module world/TouchJoystick
 * @description An on-screen stick for walking on touch screens: drag the knob
 * to move (up = forward), release to stop. Reports `(forward, strafe)` in
 * `−1..1`; looking around stays a drag on the scene.
 */

import { useRef, useState, type PointerEvent } from "react";

const RADIUS_PX = 44;

/** Stick deflection for a knob offset `(dx, dy)` px, clamped to the ring. */
export function stickFromOffset(dx: number, dy: number, radius = RADIUS_PX): { forward: number; strafe: number } {
  const len = Math.hypot(dx, dy);
  const k = len > radius ? radius / len : 1;
  return { forward: (-dy * k) / radius, strafe: (dx * k) / radius };
}

export function TouchJoystick({
  onChange,
  label,
}: {
  onChange: (forward: number, strafe: number) => void;
  label: string;
}) {
  const origin = useRef<{ x: number; y: number } | null>(null);
  const [knob, setKnob] = useState({ x: 0, y: 0 });

  const move = (e: PointerEvent<HTMLDivElement>) => {
    if (!origin.current) return;
    const s = stickFromOffset(e.clientX - origin.current.x, e.clientY - origin.current.y);
    setKnob({ x: s.strafe * RADIUS_PX, y: -s.forward * RADIUS_PX });
    onChange(s.forward, s.strafe);
  };
  const end = () => {
    origin.current = null;
    setKnob({ x: 0, y: 0 });
    onChange(0, 0);
  };

  return (
    <div
      role="application"
      aria-label={label}
      className="map:absolute map:bottom-4 map:left-4 map:w-28 map:h-28 map:rounded-full map:bg-scrim/40 map:border map:border-on-media/30 map:touch-none map:select-none"
      onPointerDown={(e) => {
        e.currentTarget.setPointerCapture(e.pointerId);
        const r = e.currentTarget.getBoundingClientRect();
        origin.current = { x: r.left + r.width / 2, y: r.top + r.height / 2 };
        move(e);
      }}
      onPointerMove={move}
      onPointerUp={end}
      onPointerCancel={end}
    >
      <div
        className="map:absolute map:w-10 map:h-10 map:rounded-full map:bg-on-media/70 map:left-1/2 map:top-1/2"
        style={{ transform: `translate(calc(-50% + ${knob.x}px), calc(-50% + ${knob.y}px))` }}
      />
    </div>
  );
}
