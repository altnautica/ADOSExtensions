import { cleanup, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

const sessions = vi.hoisted(() => ({ absent: false }));

vi.mock("../../../src/hooks/use-sessions", () => ({
  useSessions: () => ({
    client: null,
    sessions: [],
    rates: {},
    loading: false,
    absent: sessions.absent,
    stale: false,
    refresh: () => {},
  }),
}));

vi.mock("@altnautica/mapping-ui/hooks/use-compute-jobs", () => ({
  useComputeJobs: () => ({ jobs: [], client: null, unreachable: false }),
}));

vi.mock("@altnautica/mapping-ui/components/sessions/JobWorldPanel", () => ({ JobWorldPanel: () => null }));
vi.mock("@altnautica/mapping-ui/components/sessions/JobPipelineView", () => ({ JobPipelineView: () => null }));

import { SessionsView } from "../../../src/components/sessions/SessionsView";

describe("Sessions empty states", () => {
  afterEach(() => {
    cleanup();
    sessions.absent = false;
  });

  it("says no sessions are recorded on a device that has none", () => {
    render(<SessionsView />);
    expect(screen.getByText("No sessions recorded yet.")).toBeTruthy();
  });

  it("says no capture service answers when the device has none", () => {
    sessions.absent = true;
    render(<SessionsView />);
    expect(screen.getByText("No capture service is answering on this device.")).toBeTruthy();
    expect(screen.queryByText("No sessions recorded yet.")).toBeNull();
  });
});
