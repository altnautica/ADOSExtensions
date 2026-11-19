import { cleanup, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it } from "vitest";

import { GpuCard } from "../../src/pages/workstation/GpuCard";
import { useComputeStore } from "../../src/stores/compute-store";
import type { ComputeGpuInfo } from "@altnautica/mapping-ui/lib/agent/compute-client";

const NO_GPU = "No GPU on this node: reconstruction runs on the CPU and is slow.";

function report(gpu: ComputeGpuInfo | null) {
  useComputeStore.getState().setGpu("ws-1", gpu, Date.now());
}

const blank: ComputeGpuInfo = {
  present: null,
  name: null,
  cores: null,
  unifiedMemoryMb: null,
  metal: null,
  utilizationPct: null,
};

describe("GpuCard", () => {
  beforeEach(() => useComputeStore.setState({ nodes: {} }));
  afterEach(cleanup);

  it("says once that a node with no GPU reconstructs on the CPU", () => {
    report({ ...blank, present: false });
    render(<GpuCard deviceId="ws-1" />);
    expect(screen.getAllByText(NO_GPU)).toHaveLength(1);
  });

  it("names the GPU of a node that has one, without the CPU sentence", () => {
    report({ ...blank, present: true, name: "RTX 5070", utilizationPct: 40 });
    render(<GpuCard deviceId="ws-1" />);
    expect(screen.getByText("RTX 5070")).toBeTruthy();
    expect(screen.queryByText(NO_GPU)).toBeNull();
  });

  it("claims nothing about a node whose platform has no GPU probe", () => {
    report(blank);
    render(<GpuCard deviceId="ws-1" />);
    expect(screen.queryByText(NO_GPU)).toBeNull();
  });
});
