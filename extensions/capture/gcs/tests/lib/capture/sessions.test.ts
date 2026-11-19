/**
 * Session rows and upload progress: coercion of the device's session list,
 * the progress fraction (committed = complete, unknown total = indeterminate),
 * throughput between two polls and the remaining time.
 */

import { describe, expect, it } from "vitest";

import {
  coerceSessions,
  formatBytes,
  formatDuration,
  uploadEtaS,
  uploadFraction,
  uploadRate,
  type SessionUpload,
} from "../../../src/lib/capture/sessions";

const upload = (patch: Partial<SessionUpload>): SessionUpload => ({
  state: "uploading",
  bytesSent: 0,
  bytesTotal: 1000,
  node: "workstation-1",
  error: null,
  ...patch,
});

describe("coerceSessions", () => {
  it("reads rows, defaults the upload total to the session size, drops id-less rows", () => {
    const list = coerceSessions({
      sessions: [
        {
          session_id: "visor-1-20261019T101500Z-3fa2",
          duration_s: 312.4,
          size_bytes: 48213,
          frames: 12480,
          rooms: ["Dock", 7, "Aisle 1"],
          floors: [0],
          timebase: "hub_freerun",
          anchor_source: null,
          uploaded: false,
          upload: { state: "uploading", bytes_sent: 1000, node: "workstation-1" },
        },
        { duration_s: 1 },
      ],
    });
    expect(list).toHaveLength(1);
    const s = list[0]!;
    expect(s.rooms).toEqual(["Dock", "Aisle 1"]);
    expect(s.anchorSource).toBeNull();
    expect(s.upload).toEqual({ state: "uploading", bytesSent: 1000, bytesTotal: 48213, node: "workstation-1", error: null });
    expect(coerceSessions(null)).toEqual([]);
  });

  it("reads an unknown upload state as idle", () => {
    const [s] = coerceSessions({ sessions: [{ session_id: "a", upload: { state: "teleporting" } }] });
    expect(s?.upload.state).toBe("idle");
  });

  it("keeps a blocked auto-upload and its reason", () => {
    const [s] = coerceSessions({ sessions: [{ session_id: "a", upload: { state: "blocked", error: "no_credential" } }] });
    expect(s?.upload.state).toBe("blocked");
    expect(s?.upload.error).toBe("no_credential");
  });
});

describe("upload progress", () => {
  it("is the sent fraction, complete once committed, indeterminate without a total", () => {
    expect(uploadFraction(upload({ bytesSent: 250 }))).toBe(0.25);
    expect(uploadFraction(upload({ bytesSent: 5000 }))).toBe(1);
    expect(uploadFraction(upload({ state: "committed", bytesSent: 0 }))).toBe(1);
    expect(uploadFraction(upload({ bytesTotal: null }))).toBeNull();
    expect(uploadFraction(upload({ bytesTotal: 0 }))).toBeNull();
  });

  it("derives throughput from two polls and refuses a non-advancing pair", () => {
    expect(uploadRate({ bytesSent: 1000, atMs: 0 }, { bytesSent: 3000, atMs: 2000 })).toBe(1000);
    expect(uploadRate(null, { bytesSent: 3000, atMs: 2000 })).toBeNull();
    expect(uploadRate({ bytesSent: 3000, atMs: 2000 }, { bytesSent: 3000, atMs: 2000 })).toBeNull();
    expect(uploadRate({ bytesSent: 3000, atMs: 0 }, { bytesSent: 1000, atMs: 1000 })).toBeNull();
  });

  it("estimates the time left at the current rate", () => {
    expect(uploadEtaS(upload({ bytesSent: 400 }), 100)).toBe(6);
    expect(uploadEtaS(upload({ bytesSent: 400 }), null)).toBeNull();
    expect(uploadEtaS(upload({ bytesTotal: null }), 100)).toBeNull();
  });
});

describe("formatting", () => {
  it("formats sizes and durations", () => {
    expect(formatBytes(512)).toBe("512 B");
    expect(formatBytes(38.4 * 1024 ** 3)).toBe("38.4 GiB");
    expect(formatBytes(null)).toBe("–");
    expect(formatDuration(862)).toBe("14:22");
    expect(formatDuration(3725)).toBe("1:02:05");
  });
});
