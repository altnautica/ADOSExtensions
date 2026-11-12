/**
 * The Live World page for a drone that has never answered its readiness
 * poll: Start is disabled with the reason as visible text from the first
 * paint, and after enough failed polls the chip and the capture bar say the
 * drone is not answering instead of "unknown" forever.
 */

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, render, screen } from "@testing-library/react";
import type { InlineAgentApi, InlineHostApi } from "@altnautica/plugin-sdk/inline";

import messages from "../../../locales/en.json";
import { ToastProvider } from "../../src/components/ui/Toast";
import { HostProvider } from "../../src/host/context";
import {
  ATLAS_READINESS_IDLE_POLL_INTERVAL_MS,
  READINESS_FAILED_POLLS_OFFLINE,
} from "../../src/hooks/use-atlas-control";
import { LiveWorldPage } from "../../src/pages/drone/LiveWorldPage";
import { useAtlasReadinessStore } from "../../src/stores/atlas-readiness-store";
import { json } from "../helpers/agent";

// The page's world-model and cloud sources are out of scope here.
vi.mock("../../src/components/atlas/WorldModelViewport", () => ({ WorldModelViewport: () => null }));
vi.mock("../../src/hooks/use-cloud-jobs", () => ({ useCloudJobs: () => [] }));
vi.mock("../../src/hooks/use-output-frame", () => ({ useOutputFrame: () => undefined }));
vi.mock("../../src/pages/drone/use-cloud-artifact", () => ({ useCloudArtifact: () => null }));
vi.mock("../../src/hooks/use-drone-world-model", () => ({
  useDroneWorldModel: () => ({
    status: "idle",
    hasComputeNode: false,
    artifact: null,
    viewerHint: null,
    backend: null,
    output: null,
    outputs: [],
    sessions: [],
    computeNodeDeviceId: null,
    computeClient: null,
  }),
}));

const c = messages.atlas.capture;

function host(fetch: InlineAgentApi["fetch"]): InlineHostApi {
  const agent: InlineAgentApi = { fetch, websocket: () => Promise.reject(new Error("no socket")) };
  return {
    node: { deviceId: "d1", profile: null },
    agent,
    nodes: { list: () => [], agent: () => agent },
  } as unknown as InlineHostApi;
}

function show(fetch: InlineAgentApi["fetch"]) {
  render(
    <HostProvider host={host(fetch)}>
      <ToastProvider>
        <LiveWorldPage />
      </ToastProvider>
    </HostProvider>,
  );
}

const startButton = () => screen.getByRole("button", { name: c.startCapture }) as HTMLButtonElement;

/** Let the pending poll settle, then advance one idle poll interval. */
async function nextPoll() {
  await act(async () => {
    await vi.advanceTimersByTimeAsync(ATLAS_READINESS_IDLE_POLL_INTERVAL_MS);
  });
}

beforeEach(() => {
  vi.useFakeTimers();
  useAtlasReadinessStore.setState({ snapshots: {}, failedPolls: {} });
});

afterEach(() => {
  cleanup();
  vi.useRealTimers();
});

describe("LiveWorldPage for a drone that never answered", () => {
  it("disables Start with the readiness-unknown reason on first paint", async () => {
    show(() => Promise.reject(new Error("node_unreachable")));
    expect(startButton().disabled).toBe(true);
    expect(screen.getByText(c.startBlockedReadinessUnknown)).toBeTruthy();
    expect(screen.getByText(c.chip.unknown)).toBeTruthy();
  });

  it("reads \"not answering\" after repeated failed polls, not \"unknown\" forever", async () => {
    const fetch = vi.fn(() => Promise.reject(new Error("node_unreachable")));
    show(fetch);
    // The first poll runs on mount; the rest at the idle cadence.
    await act(async () => {
      await vi.advanceTimersByTimeAsync(0);
    });
    for (let i = 1; i < READINESS_FAILED_POLLS_OFFLINE - 1; i++) await nextPoll();
    expect(screen.getByText(c.chip.unknown)).toBeTruthy();
    await nextPoll();
    expect(fetch.mock.calls.length).toBeGreaterThanOrEqual(READINESS_FAILED_POLLS_OFFLINE);
    expect(screen.getByText(c.chip.notAnswering)).toBeTruthy();
    expect(screen.getByText(c.nodeNotAnswering)).toBeTruthy();
    expect(startButton().disabled).toBe(true);
  });

  it("an answer after failed polls clears the not-answering state", async () => {
    let up = false;
    const readiness = {
      enabled: true,
      profile: "default",
      capture_profile: "default",
      cameras_configured: 1,
      service_running: true,
      capturing: false,
      state: "idle",
    };
    const fetch = vi.fn(() => (up ? json(200, readiness) : Promise.reject(new Error("node_unreachable"))));
    show(fetch);
    await act(async () => {
      await vi.advanceTimersByTimeAsync(0);
    });
    for (let i = 1; i < READINESS_FAILED_POLLS_OFFLINE; i++) await nextPoll();
    expect(screen.getByText(c.chip.notAnswering)).toBeTruthy();
    up = true;
    await nextPoll();
    expect(screen.queryByText(c.chip.notAnswering)).toBeNull();
    expect(screen.queryByText(c.startBlockedReadinessUnknown)).toBeNull();
    expect(startButton().disabled).toBe(false);
    expect(useAtlasReadinessStore.getState().snapshots.d1?.readiness.enabled).toBe(true);
  });
});
