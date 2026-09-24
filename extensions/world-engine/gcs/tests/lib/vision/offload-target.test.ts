import { describe, expect, it } from "vitest";

import { nodeToOffloadAddr } from "../../../src/lib/vision/offload-target";

describe("nodeToOffloadAddr", () => {
  it("dials the node's LAN host on the compute job-API port (:8092)", () => {
    // The drone submits offload jobs to the compute node's own listener, not
    // the agent front the GCS paired to.
    expect(nodeToOffloadAddr({ lanHost: "192.0.2.5" })).toBe("192.0.2.5:8092");
    expect(nodeToOffloadAddr({ lanHost: "ws.local" })).toBe("ws.local:8092");
    // An IPv6 literal keeps its brackets so the port stays unambiguous.
    expect(nodeToOffloadAddr({ lanHost: "[fd00::5]" })).toBe("[fd00::5]:8092");
  });

  it("trims stray whitespace from the host", () => {
    expect(nodeToOffloadAddr({ lanHost: "  ws.local " })).toBe("ws.local:8092");
  });

  it("returns empty (auto-discover) for a node with no LAN host", () => {
    expect(nodeToOffloadAddr({ lanHost: null })).toBe("");
    expect(nodeToOffloadAddr({ lanHost: "" })).toBe("");
    expect(nodeToOffloadAddr({ lanHost: "   " })).toBe("");
  });
});
