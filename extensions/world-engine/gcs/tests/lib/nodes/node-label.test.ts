/**
 * A node id or offload address resolves to the paired node's name; the raw
 * id or address stays available for the title.
 */

import { describe, expect, it } from "vitest";

import { nodeLabelForAddr, nodeLabelForId } from "../../../src/lib/nodes/node-label";

const nodes = [
  { deviceId: "ws-7f3a9c", name: "Bench workstation", lanHost: "10.0.0.5" },
  { deviceId: "ws-2", name: "", lanHost: null },
];

describe("node labels", () => {
  it("names a node by id, keeping the id as the title", () => {
    expect(nodeLabelForId("ws-7f3a9c", nodes)).toEqual({ label: "Bench workstation", title: "ws-7f3a9c" });
    expect(nodeLabelForId("ws-2", nodes)).toEqual({ label: "ws-2", title: "ws-2" });
    expect(nodeLabelForId("other", nodes)).toEqual({ label: "other", title: "other" });
  });

  it("names an offload address by the node serving it", () => {
    expect(nodeLabelForAddr("10.0.0.5:8092", nodes)).toEqual({ label: "Bench workstation", title: "10.0.0.5:8092" });
    expect(nodeLabelForAddr("10.0.0.9:8092", nodes)).toEqual({ label: "10.0.0.9:8092", title: "10.0.0.9:8092" });
  });
});
