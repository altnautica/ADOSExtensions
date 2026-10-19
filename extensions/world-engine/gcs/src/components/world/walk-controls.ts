/**
 * @module world/walk-controls
 * @description First-person walking for the world viewer: WASD / arrow keys
 * and a touch joystick move the eye at a fixed height above the current floor,
 * a pointer drag looks around. Pure state + an `update(dt)` the render loop
 * calls; no DOM beyond the listeners it is handed.
 */

import type { PerspectiveCamera } from "three";

/** Eye height above the floor, metres. */
export const EYE_HEIGHT_M = 1.6;
/** Walking speed, m/s (shift doubles it). */
export const WALK_SPEED_MPS = 1.4;
const LOOK_RAD_PER_PX = 0.004;

export interface StickInput {
  /** −1 (back) … 1 (forward). */
  forward: number;
  /** −1 (left) … 1 (right). */
  strafe: number;
}

const KEY_AXES: Record<string, Partial<StickInput>> = {
  KeyW: { forward: 1 },
  ArrowUp: { forward: 1 },
  KeyS: { forward: -1 },
  ArrowDown: { forward: -1 },
  KeyA: { strafe: -1 },
  ArrowLeft: { strafe: -1 },
  KeyD: { strafe: 1 },
  ArrowRight: { strafe: 1 },
};

/** The horizontal step in viewer coordinates (x east, z south) for `input`
 * at `yaw` (radians, 0 = looking north / −z) over `dt` seconds. */
export function walkStep(input: StickInput, yaw: number, dt: number, speed = WALK_SPEED_MPS): [number, number] {
  const mag = Math.hypot(input.forward, input.strafe);
  if (mag === 0) return [0, 0];
  const scale = (Math.min(1, mag) / mag) * speed * dt;
  const f = input.forward * scale;
  const s = input.strafe * scale;
  // Forward is −z rotated by yaw about +y; right is +x rotated likewise.
  return [-Math.sin(yaw) * f + Math.cos(yaw) * s, -Math.cos(yaw) * f - Math.sin(yaw) * s];
}

/** Whether a key event should move the walker: only when the viewer element
 * itself is the focus target, and never from an editable or a select. */
export function walkKeyApplies(eventTarget: EventTarget | null, viewer: HTMLElement): boolean {
  if (eventTarget !== viewer) return false;
  const el = eventTarget as HTMLElement;
  return !(
    el instanceof HTMLInputElement ||
    el instanceof HTMLTextAreaElement ||
    el instanceof HTMLSelectElement ||
    el.isContentEditable
  );
}

export class WalkControls {
  yaw = 0;
  pitch = 0;
  floorY = 0;
  stick: StickInput = { forward: 0, strafe: 0 };
  private keys = new Set<string>();
  private fast = false;
  private drag: { x: number; y: number } | null = null;
  private readonly off: (() => void)[] = [];

  constructor(
    private readonly camera: PerspectiveCamera,
    target: HTMLElement,
  ) {
    // Keys move the walker only while the viewer itself has focus, never
    // while the operator types in a field or uses a control elsewhere.
    if (target.tabIndex < 0) target.tabIndex = 0;
    const down = (e: KeyboardEvent) => {
      if (!walkKeyApplies(e.target, target)) return;
      if (KEY_AXES[e.code]) {
        this.keys.add(e.code);
        e.preventDefault();
      }
      this.fast = e.shiftKey;
    };
    const up = (e: KeyboardEvent) => {
      this.keys.delete(e.code);
      this.fast = e.shiftKey;
    };
    const blur = () => this.keys.clear();
    const pdown = (e: PointerEvent) => {
      target.focus({ preventScroll: true });
      this.drag = { x: e.clientX, y: e.clientY };
    };
    const pmove = (e: PointerEvent) => {
      if (!this.drag) return;
      this.yaw += (e.clientX - this.drag.x) * LOOK_RAD_PER_PX;
      this.pitch = Math.max(-1.2, Math.min(1.2, this.pitch - (e.clientY - this.drag.y) * LOOK_RAD_PER_PX));
      this.drag = { x: e.clientX, y: e.clientY };
    };
    const pup = () => {
      this.drag = null;
    };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    target.addEventListener("pointerdown", pdown);
    target.addEventListener("blur", blur);
    window.addEventListener("pointermove", pmove);
    window.addEventListener("pointerup", pup);
    this.off.push(
      () => window.removeEventListener("keydown", down),
      () => window.removeEventListener("keyup", up),
      () => target.removeEventListener("pointerdown", pdown),
      () => target.removeEventListener("blur", blur),
      () => window.removeEventListener("pointermove", pmove),
      () => window.removeEventListener("pointerup", pup),
    );
  }

  /** Start walking from the camera's current spot, looking the way it looks. */
  enter(floorY: number): void {
    this.floorY = floorY;
    const dir = this.camera.getWorldDirection(this.camera.position.clone());
    this.yaw = Math.atan2(-dir.x, -dir.z);
    this.pitch = 0;
    this.camera.position.y = floorY + EYE_HEIGHT_M;
  }

  update(dt: number): void {
    const input: StickInput = { ...this.stick };
    for (const code of this.keys) {
      const axis = KEY_AXES[code];
      input.forward += axis?.forward ?? 0;
      input.strafe += axis?.strafe ?? 0;
    }
    const [dx, dz] = walkStep(input, this.yaw, dt, WALK_SPEED_MPS * (this.fast ? 2 : 1));
    this.camera.position.x += dx;
    this.camera.position.z += dz;
    this.camera.position.y = this.floorY + EYE_HEIGHT_M;
    this.camera.rotation.set(this.pitch, this.yaw, 0, "YXZ");
  }

  dispose(): void {
    for (const f of this.off) f();
    this.keys.clear();
  }
}
