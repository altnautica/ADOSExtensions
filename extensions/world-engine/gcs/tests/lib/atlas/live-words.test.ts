/**
 * The Live World header chip never claims "idle" for a drone it has not heard
 * from, and every capture-state word comes from the catalogue.
 */

import { describe, expect, it } from "vitest";

import messages from "../../../../locales/en.json";
import { captureChip, stateWordKey } from "../../../src/lib/atlas/live-words";
import { translator } from "../../../src/host/i18n";

const t = translator("atlas");

describe("captureChip", () => {
  it("reads unknown when nothing answered yet", () => {
    const chip = captureChip({ liveState: null, hasLive: false, isStale: true, readinessState: null, offline: false });
    expect(t(chip.key)).toBe(messages.atlas.capture.chip.unknown);
  });

  it("reads not answering once the node stopped answering", () => {
    const chip = captureChip({ liveState: null, hasLive: false, isStale: true, readinessState: null, offline: true });
    expect(t(chip.key)).toBe(messages.atlas.capture.chip.notAnswering);
    expect(chip.tone).toBe("we:text-status-error");
  });

  it("reads the live state while telemetry is fresh, else the readiness state", () => {
    expect(
      t(captureChip({ liveState: "capturing", hasLive: true, isStale: false, readinessState: null, offline: false }).key),
    ).toBe(messages.atlas.capture.state.capturing);
    expect(
      t(captureChip({ liveState: null, hasLive: false, isStale: true, readinessState: "idle", offline: false }).key),
    ).toBe(messages.atlas.capture.state.idle);
  });

  it("reads stale for a frozen heartbeat", () => {
    const chip = captureChip({ liveState: "capturing", hasLive: true, isStale: true, readinessState: null, offline: false });
    expect(t(chip.key)).toBe(messages.atlas.stale);
  });
});

describe("stateWordKey", () => {
  it("keys every agent state and falls back to Other", () => {
    for (const s of ["capturing", "idle", "paused", "finalizing", "bagged"]) {
      expect(t(stateWordKey(s))).toBe((messages.atlas.capture.state as Record<string, string>)[s]);
    }
    expect(t(stateWordKey("warp"))).toBe(messages.atlas.capture.state.other);
  });
});
