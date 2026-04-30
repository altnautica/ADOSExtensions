import { describe, it, expect } from "vitest";

import {
  formatAmps,
  formatCelsius,
  formatEta,
  formatMilliVolts,
  formatPercent,
  formatVolts,
} from "../src/formatters";

describe("formatters", () => {
  it("formats volts with default 2 digits and a unit suffix", () => {
    expect(formatVolts(15.6)).toBe("15.60 V");
    expect(formatVolts(15.6, 1)).toBe("15.6 V");
  });

  it("returns an em dash for non-finite numbers", () => {
    expect(formatVolts(Number.NaN)).toBe("—");
    expect(formatAmps(Number.POSITIVE_INFINITY)).toBe("—");
    expect(formatPercent(-1)).toBe("—");
    expect(formatCelsius(null)).toBe("—");
  });

  it("rounds millivolts to integers", () => {
    expect(formatMilliVolts(67.4)).toBe("67 mV");
    expect(formatMilliVolts(67.6)).toBe("68 mV");
  });

  it("formats ETA in seconds, minutes, or hours depending on magnitude", () => {
    expect(formatEta(0)).toBe("0s");
    expect(formatEta(45)).toBe("45s");
    expect(formatEta(120)).toBe("2m 00s");
    expect(formatEta(3725)).toBe("1h 02m");
  });

  it("returns an em dash for null or negative ETA", () => {
    expect(formatEta(null)).toBe("—");
    expect(formatEta(-5)).toBe("—");
  });
});
