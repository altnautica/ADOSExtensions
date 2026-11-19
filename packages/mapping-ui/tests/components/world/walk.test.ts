/**
 * First-person walking: the step for a stick input at a heading, and the
 * touch stick's deflection clamped to its ring.
 */

import { describe, expect, it } from "vitest";

import { stickFromOffset } from "../../../src/components/world/TouchJoystick";
import { walkStep, WALK_SPEED_MPS } from "../../../src/components/world/walk-controls";

describe("walkStep", () => {
  it("walks north (−z) when facing north and pushing forward", () => {
    const [dx, dz] = walkStep({ forward: 1, strafe: 0 }, 0, 1);
    expect(dx).toBeCloseTo(0, 12);
    expect(dz).toBeCloseTo(-WALK_SPEED_MPS, 12);
  });

  it("strafes east (+x) when facing north", () => {
    const [dx, dz] = walkStep({ forward: 0, strafe: 1 }, 0, 1);
    expect(dx).toBeCloseTo(WALK_SPEED_MPS, 12);
    expect(dz).toBeCloseTo(0, 12);
  });

  it("walks west (−x) after turning left a quarter turn", () => {
    const [dx, dz] = walkStep({ forward: 1, strafe: 0 }, Math.PI / 2, 1);
    expect(dx).toBeCloseTo(-WALK_SPEED_MPS, 12);
    expect(dz).toBeCloseTo(0, 12);
  });

  it("never exceeds walking speed on a diagonal and stands still at rest", () => {
    const [dx, dz] = walkStep({ forward: 1, strafe: 1 }, 0.3, 1);
    expect(Math.hypot(dx, dz)).toBeCloseTo(WALK_SPEED_MPS, 12);
    expect(walkStep({ forward: 0, strafe: 0 }, 0, 1)).toEqual([0, 0]);
  });
});

describe("stickFromOffset", () => {
  it("maps knob up to forward and right to strafe, clamped to the ring", () => {
    expect(stickFromOffset(0, -22, 44)).toEqual({ forward: 0.5, strafe: 0 });
    const far = stickFromOffset(88, 0, 44);
    expect(far.strafe).toBeCloseTo(1, 12);
    expect(far.forward === 0).toBe(true);
  });
});
