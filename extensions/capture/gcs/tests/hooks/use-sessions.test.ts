/**
 * A missed sessions poll keeps the last good list and its rates (so the page
 * and an open viewer stay mounted) and only counts toward staleness.
 */

import { describe, expect, it } from "vitest";

import { STALE_AFTER_MISSES } from "../../src/hooks/use-capture";
import { applySessionsPoll, EMPTY_SESSIONS_POLL } from "../../src/hooks/use-sessions";
import type { CaptureSession } from "../../src/lib/capture/sessions";

function session(bytesSent: number): CaptureSession {
  return {
    sessionId: "s1",
    upload: { state: "uploading", bytesSent },
  } as unknown as CaptureSession;
}

describe("applySessionsPoll", () => {
  it("keeps the last good list and rates through a failed poll", () => {
    let p = applySessionsPoll(EMPTY_SESSIONS_POLL, [session(0)], 0);
    p = applySessionsPoll(p, [session(3000)], 3000);
    const good = p.memory.value;
    const rates = p.rates;
    expect(good).toHaveLength(1);
    expect(Object.keys(rates)).toEqual(["s1"]);

    p = applySessionsPoll(p, null, 6000);
    expect(p.memory.value).toBe(good);
    expect(p.rates).toBe(rates);
    expect(p.memory.misses).toBe(1);
  });

  it("counts misses toward staleness and clears them on the next reply", () => {
    let p = applySessionsPoll(EMPTY_SESSIONS_POLL, [session(0)], 0);
    for (let i = 0; i < STALE_AFTER_MISSES; i++) p = applySessionsPoll(p, null, 1000 * (i + 1));
    expect(p.memory.misses).toBe(STALE_AFTER_MISSES);
    expect(p.memory.value).toHaveLength(1);
    p = applySessionsPoll(p, [session(10)], 9000);
    expect(p.memory.misses).toBe(0);
  });

  it("stays empty when the device never answered", () => {
    expect(applySessionsPoll(EMPTY_SESSIONS_POLL, null, 0).memory.value).toBeNull();
  });
});
