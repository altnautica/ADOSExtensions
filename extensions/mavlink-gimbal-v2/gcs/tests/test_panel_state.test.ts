/**
 * Panel behaviour against live state: staleness, the ROI controls in a
 * sandbox without form submission, slider seeding, telemetry-confirmed ROI
 * status, and operator words for every string.
 */

import { readFileSync } from "node:fs";
import { resolve } from "node:path";

import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import en from "../../locales/en.json";
import { mountPanel } from "../src/panel";
import type { GimbalState } from "../src/types";

let rootEl: HTMLElement;

interface OuterArgs {
  command: string;
  args: { key: string; value: unknown };
}

beforeEach(() => {
  document.body.innerHTML = "";
  rootEl = document.createElement("div");
  document.body.appendChild(rootEl);
});

afterEach(() => {
  vi.useRealTimers();
  document.body.innerHTML = "";
});

async function harness() {
  const h = createPluginHarness({
    grantedCapabilities: ["command.send"],
    locale: en,
    mount: () => undefined,
  });
  await h.start();
  return h;
}

// Each agent sample carries its own stamp; the panel ignores a repeat.
let seq = 0;
function sample(over: Partial<GimbalState> = {}): GimbalState {
  seq += 1;
  return {
    timestampMs: Date.now() + seq,
    pitchDeg: -20,
    yawDeg: 30,
    rollDeg: 0,
    mode: "manual",
    ...over,
  };
}

const flush = () => new Promise((r) => setTimeout(r, 0));

function lastWrite(calls: readonly { args: unknown }[]): OuterArgs["args"] {
  return (calls[calls.length - 1]!.args as OuterArgs).args;
}

describe("stale gimbal state", () => {
  it("hides the angles once the feed has been silent for two seconds", async () => {
    vi.useFakeTimers({ toFake: ["Date", "setInterval", "clearInterval"] });
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState(sample());
    const readout = rootEl.querySelector<HTMLElement>(".agm-readout")!;
    expect(readout.hidden).toBe(false);
    expect(rootEl.textContent).not.toContain("Gimbal not reporting");
    vi.advanceTimersByTime(3000);
    expect(readout.hidden).toBe(true);
    expect(rootEl.textContent).toContain("Gimbal not reporting");
    expect(rootEl.textContent).toContain("3 s");
    panel.destroy();
    await h.teardown();
  });

  it("shows a payload whose host stamp is five seconds old as stale on first paint", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState({ ...sample(), ts_ms: Date.now() - 5000 });
    expect(rootEl.querySelector<HTMLElement>(".agm-readout")!.hidden).toBe(true);
    expect(rootEl.textContent).toContain("Gimbal not reporting");
    expect(rootEl.textContent).not.toContain("-20.0 deg");
    panel.destroy();
    await h.teardown();
  });

  it("stays live at the host's 750 ms cadence with jitter and skipped ticks", async () => {
    vi.useFakeTimers({ toFake: ["Date", "setInterval", "clearInterval"] });
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    const readout = rootEl.querySelector<HTMLElement>(".agm-readout")!;
    // The agent writes at most every 500 ms, so a delivered sample is up to
    // ~450 ms old by the host stamp. Gaps include 1500 ms skipped ticks.
    const gaps = [750, 810, 690, 1500, 760, 740, 1500, 1500, 700, 800];
    const ageOnArrival = [0, 450, 120, 450, 300, 50, 400, 450, 200, 10];
    let n = 0;
    for (const gap of gaps) {
      const stamp = Date.now() - ageOnArrival[n]!;
      panel.setState({ ...sample({ timestampMs: stamp }), ts_ms: stamp });
      // Check the readout at every 50 ms step until the next sample.
      for (let t = 0; t < gap; t += 50) {
        vi.advanceTimersByTime(50);
        expect(readout.hidden, `sample ${n} +${t + 50} ms`).toBe(false);
      }
      n += 1;
    }
    panel.destroy();
    await h.teardown();
  });

  it("stays live when the agent clock runs 1.5 s behind the browser", async () => {
    vi.useFakeTimers({ toFake: ["Date", "setInterval", "clearInterval"] });
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    const readout = rootEl.querySelector<HTMLElement>(".agm-readout")!;
    for (const gap of [750, 1500, 750, 1500, 750]) {
      panel.setState(sample({ timestampMs: Date.now() - 1500 }));
      expect(readout.hidden).toBe(false);
      vi.advanceTimersByTime(gap - 1);
      expect(readout.hidden).toBe(false);
      vi.advanceTimersByTime(1);
    }
    panel.destroy();
    await h.teardown();
  });

  it("does not reset the age when the same sample is delivered again", async () => {
    vi.useFakeTimers({ toFake: ["Date", "setInterval", "clearInterval"] });
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    const readout = rootEl.querySelector<HTMLElement>(".agm-readout")!;
    const first = sample();
    panel.setState(first);
    for (let i = 0; i < 3; i += 1) {
      vi.advanceTimersByTime(750);
      panel.setState({ ...first });
    }
    // 2250 ms since the one real sample: stale despite the repeats.
    expect(readout.hidden).toBe(true);
    panel.destroy();
    await h.teardown();
  });
});

describe("ROI controls without form submission", () => {
  it("has no form and locks from a plain button click", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    expect(rootEl.querySelector("form")).toBeNull();
    const lock = rootEl.querySelector<HTMLButtonElement>(".agm-roi-submit")!;
    expect(lock.type).toBe("button");
    rootEl.querySelector<HTMLInputElement>(".agm-roi-lat")!.value = "12.5";
    rootEl.querySelector<HTMLInputElement>(".agm-roi-lon")!.value = "77.25";
    rootEl.querySelector<HTMLInputElement>(".agm-roi-alt")!.value = "40";
    lock.click();
    await flush();
    expect(lastWrite(h.calls)).toEqual({
      key: "roi",
      value: { lat_deg: 12.5, lon_deg: 77.25, alt_m: 40 },
    });
    panel.destroy();
    await h.teardown();
  });

  it("locks when Enter is pressed in a field", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    const lat = rootEl.querySelector<HTMLInputElement>(".agm-roi-lat")!;
    lat.value = "1.5";
    lat.dispatchEvent(new KeyboardEvent("keydown", { key: "Enter", bubbles: true }));
    await flush();
    expect(lastWrite(h.calls)).toMatchObject({ key: "roi", value: { lat_deg: 1.5 } });
    panel.destroy();
    await h.teardown();
  });
});

describe("slider seeding", () => {
  it("sends the untouched axis at its live angle, not zero", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState(sample({ pitchDeg: -20, yawDeg: 30 }));
    const pitch = rootEl.querySelector<HTMLInputElement>(".agm-pitch")!;
    expect(pitch.value).toBe("-20");
    pitch.value = "-40";
    pitch.dispatchEvent(new Event("input"));
    // A sample during the drag must not yank the slider back.
    panel.setState(sample({ pitchDeg: -21, yawDeg: 31 }));
    expect(pitch.value).toBe("-40");
    pitch.dispatchEvent(new Event("change"));
    await flush();
    expect(lastWrite(h.calls)).toEqual({ key: "point", value: { pitch_deg: -40, yaw_deg: 31 } });
    panel.destroy();
    await h.teardown();
  });
});

describe("slider release", () => {
  it("keeps following the live angle after a release at the start value", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState(sample({ pitchDeg: -20 }));
    const pitch = rootEl.querySelector<HTMLInputElement>(".agm-pitch")!;
    // Drag away and back, then let go: no change event fires.
    pitch.value = "-30";
    pitch.dispatchEvent(new Event("input"));
    pitch.value = "-20";
    pitch.dispatchEvent(new Event("input"));
    pitch.dispatchEvent(new Event("pointerup"));
    panel.setState(sample({ pitchDeg: -25 }));
    expect(pitch.value).toBe("-25");
    for (const ev of ["pointercancel", "blur"]) {
      pitch.dispatchEvent(new Event("input"));
      pitch.dispatchEvent(new Event(ev));
      panel.setState(sample({ pitchDeg: ev === "blur" ? -10 : -15 }));
      expect(pitch.value, ev).toBe(ev === "blur" ? "-10" : "-15");
    }
    panel.destroy();
    await h.teardown();
  });
});

describe("ROI status", () => {
  it("refuses an out-of-range latitude or longitude without sending", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState(sample());
    const status = () => rootEl.querySelector(".agm-roi .agm-roi-status")!.textContent;
    const before = h.calls.length;
    await panel.submitRoi({ latDeg: 95, lonDeg: 10, altM: 50 });
    expect(status()).toBe(en["roi.invalid"]);
    await panel.submitRoi({ latDeg: 10, lonDeg: -181, altM: 50 });
    expect(status()).toBe(en["roi.invalid"]);
    expect(h.calls.length).toBe(before);
    panel.destroy();
    await h.teardown();
  });

  it("does not claim a re-lock while already locked from the old lock", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState(sample({ mode: "roi-lock" }));
    await panel.submitRoi({ latDeg: 12.971, lonDeg: 77.594, altM: 50 });
    const status = () => rootEl.querySelector(".agm-roi .agm-roi-status")!.textContent;
    expect(status()).toBe("ROI sent");
    panel.setState(sample({ mode: "roi-lock" }));
    expect(status()).toBe("ROI sent");
    panel.destroy();
    await h.teardown();
  });

  it("says the request was sent, then confirms from the telemetry mode", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState(sample());
    await panel.submitRoi({ latDeg: 12.971, lonDeg: 77.594, altM: 50 });
    const status = () => rootEl.querySelector(".agm-roi .agm-roi-status")!.textContent;
    expect(status()).toBe("ROI sent");
    panel.setState(sample({ mode: "roi-lock" }));
    expect(status()).toBe("Locked on point");
    await panel.releaseRoi();
    expect(status()).toBe("Release sent");
    panel.setState(sample({ mode: "manual" }));
    expect(status()).toBe("ROI released");
    panel.destroy();
    await h.teardown();
  });

  it("says no gimbal is connected when no state has arrived", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    await panel.submitRoi({ latDeg: 1, lonDeg: 2, altM: 3 });
    expect(rootEl.querySelector(".agm-roi .agm-roi-status")!.textContent).toBe(
      "No gimbal connected",
    );
    panel.destroy();
    await h.teardown();
  });
});

describe("operator words", () => {
  it("labels the ROI altitude as above home", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    expect(rootEl.textContent).toContain("Altitude above home (m)");
    expect(rootEl.textContent).not.toContain("AGL");
    panel.destroy();
    await h.teardown();
  });

  it("shows modes as words from the locale", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    const mode = () => rootEl.querySelector(".agm-mode")!.textContent;
    const words: Record<GimbalState["mode"], string> = {
      manual: "Manual",
      "roi-lock": "Locked on point",
      "follow-me": "Following",
      neutral: "Neutral",
    };
    for (const [id, word] of Object.entries(words)) {
      panel.setState(sample({ mode: id as GimbalState["mode"] }));
      expect(mode()).toBe(word);
    }
    panel.destroy();
    await h.teardown();
  });

  it("shows a mode it does not know as unknown, not neutral", async () => {
    const h = await harness();
    const panel = mountPanel(h.ctx, rootEl);
    panel.setState(sample({ mode: "gpb-track" as GimbalState["mode"] }));
    expect(rootEl.querySelector(".agm-mode")!.textContent).toBe("Unknown");
    panel.destroy();
    await h.teardown();
  });

  it("resolves every rendered string through the locale", async () => {
    const h = createPluginHarness({
      grantedCapabilities: ["command.send"],
      locale: { ...en, "roi.lock": "LOCALIZED-LOCK", "panel.awaitingState": "LOCALIZED-WAIT" },
      mount: () => undefined,
    });
    await h.start();
    const panel = mountPanel(h.ctx, rootEl);
    expect(rootEl.textContent).toContain("LOCALIZED-LOCK");
    expect(rootEl.textContent).toContain("LOCALIZED-WAIT");
    panel.destroy();
    await h.teardown();
  });

  it("ships no locale key the panel or manifest does not use", () => {
    const ext = resolve(__dirname, "../..");
    const sources = [
      readFileSync(resolve(ext, "gcs/src/panel.ts"), "utf8"),
      readFileSync(resolve(ext, "manifest.yaml"), "utf8"),
    ].join("\n");
    const unused = Object.keys(en).filter((k) => !sources.includes(`"${k}"`));
    expect(unused).toEqual([]);
  });
});
