import { describe, expect, it } from "vitest";

import { splatFramingBounds, type ForEachSplat } from "../../../src/lib/world/splat-bounds";

type Splat = { c: [number, number, number]; s: [number, number, number]; o: number };

function walk(splats: readonly Splat[]): ForEachSplat {
  return (cb) =>
    splats.forEach((p, i) => cb(i, { x: p.c[0], y: p.c[1], z: p.c[2] }, { x: p.s[0], y: p.s[1], z: p.s[2] }, null, p.o));
}

/** A deterministic pseudo-random value in [-1, 1). */
function rand(i: number, k: number): number {
  const x = Math.sin(i * 12.9898 + k * 78.233) * 43758.5453;
  return 2 * (x - Math.floor(x)) - 1;
}

const SOLID: Splat[] = Array.from({ length: 2000 }, (_, i) => ({
  c: [3 + rand(i, 0), rand(i, 1), -5 + 2 * rand(i, 2)],
  s: [0.02, 0.03, 0.05],
  o: 0.8,
}));

describe("splatFramingBounds", () => {
  it("frames the solid splats, not the floaters (the empty-view case)", () => {
    const far = (i: number): [number, number, number] => [1e6 * rand(i, 3), 6e6 * rand(i, 4), 1e6 * rand(i, 5)];
    const floaters: Splat[] = [
      // Transparent, scattered to a million units.
      ...Array.from({ length: 5000 }, (_, i): Splat => ({ c: far(i), s: [0.1, 0.1, 0.1], o: 0.001 })),
      // Opaque but flattened to no thickness and huge across, at the SPZ limit.
      ...Array.from({ length: 600 }, (_, i): Splat => ({ c: [2048 * rand(i, 6), -2048, 2048], s: [4.5e-5, 50, 150], o: 0.9 })),
      // Centres past float16 range.
      { c: [Infinity, -Infinity, 0], s: [1, 1, 1], o: 1 },
      // A few solid strays: trimmed by the quantile.
      ...Array.from({ length: 10 }, (_, i): Splat => ({ c: [500 + i, 500, 500], s: [0.05, 0.05, 0.05], o: 0.9 })),
    ];
    const all = [...SOLID, ...floaters];
    const f = splatFramingBounds(walk(all), all.length);
    expect(f).not.toBeNull();
    const { min, max, solid } = f!;
    expect(solid).toBe(true);
    expect(min[0]).toBeGreaterThan(1.9);
    expect(max[0]).toBeLessThan(4.1);
    expect(min[1]).toBeGreaterThan(-1.1);
    expect(max[1]).toBeLessThan(1.1);
    expect(min[2]).toBeGreaterThan(-7.1);
    expect(max[2]).toBeLessThan(-2.9);
    expect(max[0] - min[0]).toBeGreaterThan(1.8);
  });

  it("reports a fully transparent splat as not solid, framed on every finite centre", () => {
    const faint = SOLID.map((p) => ({ ...p, o: 0.01 }));
    const { min, max, solid } = splatFramingBounds(walk(faint), faint.length)!;
    expect(solid).toBe(false);
    expect(min[0]).toBeGreaterThan(1.9);
    expect(max[2]).toBeLessThan(-2.9);
  });

  it("counts a lone solid splat that sampling skips", () => {
    const n = 1_000_000;
    const forEach: ForEachSplat = (cb) => {
      const c = { x: 0, y: 0, z: 0 };
      const s = { x: 0.01, y: 0.01, z: 0.01 };
      for (let i = 0; i < n; i++) {
        c.x = i === 1 ? 7 : (i / n) * 1e5;
        cb(i, c, s, null, i === 1 ? 0.9 : 0.001);
      }
    };
    const { min, max, solid } = splatFramingBounds(forEach, n)!;
    expect(solid).toBe(true);
    expect(min).toEqual([7, 0, 0]);
    expect(max).toEqual([7, 0, 0]);
  });

  it("samples a large splat evenly", () => {
    const n = 1_000_000;
    const forEach: ForEachSplat = (cb) => {
      const c = { x: 0, y: 0, z: 0 };
      const s = { x: 0.01, y: 0.01, z: 0.01 };
      for (let i = 0; i < n; i++) {
        c.x = (i / n) * 10;
        c.y = 1;
        c.z = -1;
        cb(i, c, s, null, 1);
      }
    };
    const { min, max } = splatFramingBounds(forEach, n)!;
    expect(min[0]).toBeCloseTo(0.1, 1);
    expect(max[0]).toBeCloseTo(9.9, 1);
    expect(min[1]).toBe(1);
    expect(max[2]).toBe(-1);
  });

  it("is null with no finite centre", () => {
    expect(splatFramingBounds(walk([]), 0)).toBeNull();
    expect(splatFramingBounds(walk([{ c: [NaN, 0, 0], s: [1, 1, 1], o: 1 }]), 1)).toBeNull();
  });
});
