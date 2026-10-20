/**
 * Navigation nodes: the viewer draws a disc per node of the tour and a click
 * jumps the walker there; in dollhouse mode the discs are hidden.
 */

import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";

import messages from "../../../../locales/en.json";
import { SparkViewer } from "../../../src/components/world/SparkViewer";
import type { WorldScene } from "../../../src/components/world/use-spark-scene";

// The stage stands in for the WebGL one; the mock factory is hoisted above
// the imports, so the stage is too.
const stage = vi.hoisted(() => ({
  setFloor: vi.fn(),
  setAisles: vi.fn(),
  setMode: vi.fn(),
  setMeasure: vi.fn(),
  onFrame: vi.fn(() => () => undefined),
  project: vi.fn(() => ({ x: 10, y: 20, visible: true })),
  camera: { position: { set: vi.fn(), toArray: () => [0, 0, 0] }, rotation: { y: 0 } },
}));

vi.mock("../../../src/components/world/use-spark-scene", () => ({
  useSparkScene: () => ({ stage, load: { phase: "ready" } }),
}));

afterEach(cleanup);

const scene: WorldScene = {
  splat: { key: "s", name: "scene.spz", fetch: () => Promise.reject(new Error("unused")) },
  placement: null,
  collision: null,
  floorplan: null,
  rooms: [],
  nodes: [
    { id: "n0", position: [0, 0, 1.6], yawDeg: 0 },
    { id: "n1", position: [3, 4, 1.6], yawDeg: 90 },
  ],
};

const label = (id: string) => messages.world.navNode.replace("{id}", id);

describe("navigation nodes", () => {
  it("nav_node_click_jumps", () => {
    render(<SparkViewer scene={scene} />);
    fireEvent.click(screen.getByRole("button", { name: label("n1") }));
    expect(stage.setMode).toHaveBeenLastCalledWith("walk");
    // ENU (3, 4) is viewer (3, −4); the eye stands 1.6 m above the floor.
    expect(stage.camera.position.set).toHaveBeenLastCalledWith(3, 1.6, -4);
  });

  it("hides the discs in dollhouse mode", () => {
    render(<SparkViewer scene={scene} />);
    expect(screen.getAllByRole("button", { name: /n\d/ })).toHaveLength(2);
    fireEvent.click(screen.getByRole("button", { name: messages.world.mode.dollhouse }));
    expect(screen.queryByRole("button", { name: label("n0") })).toBeNull();
  });
});
