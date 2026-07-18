import { describe, expect, it } from "vitest";

import { maxZoom, showControl, supports } from "../src/capability";
import type { PodCapabilities, PodState } from "../src/types";

function state(caps: Partial<PodCapabilities>): PodState {
  return {
    model: "test",
    known: true,
    connected: true,
    firmware: null,
    capabilities: caps,
    yaw_deg: null,
    pitch_deg: null,
    roll_deg: null,
    zoom: null,
    sensor_mode: "eo",
    gimbal_mode: "follow",
    palette: null,
    recording: false,
    laser_range_m: null,
    spot_temp_c: null,
    track_active: false,
    track_id: null,
    link_ok: true,
    frames_received: 0,
  };
}

describe("capability gating", () => {
  it("a ZT30-like profile shows every control", () => {
    const s = state({
      gimbal: true,
      zoom: true,
      thermal: true,
      laser: true,
      ai_track: true,
      max_zoom: 180,
    });
    for (const f of ["gimbal", "zoom", "thermal", "laser", "ai_track"] as const) {
      expect(showControl(s, f)).toBe(true);
    }
    expect(maxZoom(s)).toBe(180);
  });

  it("an A2-mini-like profile hides everything model-specific", () => {
    const s = state({ gimbal: false, zoom: false, thermal: false, laser: false, ai_track: false });
    for (const f of ["gimbal", "zoom", "thermal", "laser", "ai_track"] as const) {
      expect(showControl(s, f)).toBe(false);
    }
    expect(maxZoom(s)).toBe(1);
  });

  it("an A8-mini-like profile shows zoom + track but not thermal/laser", () => {
    const s = state({ gimbal: true, zoom: true, ai_track: true, thermal: false, laser: false, max_zoom: 6 });
    expect(supports(s, "zoom")).toBe(true);
    expect(supports(s, "ai_track")).toBe(true);
    expect(supports(s, "thermal")).toBe(false);
    expect(supports(s, "laser")).toBe(false);
    expect(maxZoom(s)).toBe(6);
  });

  it("a null state supports nothing", () => {
    expect(showControl(null, "gimbal")).toBe(false);
    expect(maxZoom(null)).toBe(1);
  });
});
