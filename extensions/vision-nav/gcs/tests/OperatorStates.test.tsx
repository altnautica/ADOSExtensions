import { readdirSync, readFileSync, statSync } from "node:fs";
import { join, resolve } from "node:path";

import { act, cleanup, fireEvent, render, screen, within } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

import en from "../../locales/en.json";
import { NavigationTab } from "../src/components/NavigationTab";
import { normalizeNavState, type VisionNavTelemetry } from "../src/types";

import { harness } from "./harness";

const SRC = resolve(__dirname, "../src");
const MANIFEST = resolve(__dirname, "../../manifest.yaml");

function mkTelemetry(over: Partial<VisionNavTelemetry> = {}): VisionNavTelemetry {
  return {
    opticalFlowSupported: true,
    flowQuality: 180,
    flowRateHz: 30,
    flowDistanceM: 1.25,
    companionState: "active",
    ...over,
  };
}

const LIVE_SNAPSHOT = {
  opticalFlowSupported: true,
  companionState: "active",
  mode: "external_nav",
  availableEstimators: ["off", "external_nav"],
  estimatorState: "converged",
  flowQuality: 186,
  preArmReport: {
    mode: "external_nav",
    armable: true,
    checks: [{ id: "extnav_health", severity: "ok", detail: "" }],
  },
};

const LIVE_NAV = { mode: "external_nav", active_set: 1, extnav_health: "good", odom_rate_hz: 10 };

function flatten(tree: Record<string, unknown>, prefix: string): string[] {
  return Object.entries(tree).flatMap(([k, v]) =>
    typeof v === "string" ? [`${prefix}${k}`] : flatten(v as Record<string, unknown>, `${prefix}${k}.`),
  );
}
const EN_KEYS = new Set(flatten(en as Record<string, unknown>, "navigation."));

function sourceText(dir: string): string {
  return readdirSync(dir)
    .map((name) => {
      const p = join(dir, name);
      if (statSync(p).isDirectory()) return sourceText(p);
      return /\.tsx?$/.test(name) ? readFileSync(p, "utf8") : "";
    })
    .join("\n");
}

afterEach(() => {
  cleanup();
  vi.useRealTimers();
  try {
    window.localStorage.clear();
  } catch {
    // storage stubbed out by a test
  }
});

describe("live feed freshness", () => {
  it("dims the tab and replaces the pills with the age once the feed is 2 s old", () => {
    vi.useFakeTimers();
    const h = harness();
    render(<NavigationTab ctx={h.ctx} firmware="ardupilot" />);
    act(() => {
      h.emitTelemetry("navigation", LIVE_SNAPSHOT);
      h.emitEvent("nav.state", LIVE_NAV);
    });
    const pill = screen.getByTestId("vn-companion-pill");
    expect(pill.textContent).toContain("Active");
    expect(screen.getByTestId("vn-pre-arm-armable")).toBeTruthy();
    expect(screen.getByTestId("vn-nav-extnav-health").textContent).toContain("Good");

    act(() => {
      vi.advanceTimersByTime(1500);
    });
    expect(pill.textContent).toContain("Active");

    act(() => {
      vi.advanceTimersByTime(1000);
    });
    expect(pill.dataset.state).toBe("stale");
    expect(pill.textContent).toContain("No data · 2 s");
    expect(pill.textContent).not.toContain("Active");
    expect(screen.queryByTestId("vn-pre-arm-armable")).toBeNull();
    expect(screen.getByTestId("vn-pre-arm-stale").textContent).toBe("No data · 2 s");
    expect(screen.getByTestId("vn-mode-card").textContent).not.toContain("Converged");
    expect(screen.getByTestId("vn-estimator-card").textContent).not.toContain("Converged");
    expect(screen.getByTestId("vn-estimator-card").textContent).toContain("No data · 2 s");
    expect(screen.getByTestId("vn-nav-extnav-health").textContent).toBe("No data · 2 s");
    expect(screen.getByTestId("vn-telemetry-body").dataset.stale).toBe("true");
    expect(screen.getByTestId("vn-nav-state").dataset.stale).toBe("true");

    act(() => {
      vi.advanceTimersByTime(3000);
    });
    expect(pill.textContent).toContain("No data · 5 s");

    act(() => {
      h.emitTelemetry("navigation", LIVE_SNAPSHOT);
      h.emitEvent("nav.state", LIVE_NAV);
    });
    expect(pill.textContent).toContain("Active");
    expect(screen.getByTestId("vn-telemetry-body").dataset.stale).toBe("false");
    expect(screen.getByTestId("vn-nav-extnav-health").textContent).toContain("Good");
  });
});

describe("rangefinder health", () => {
  function rfPill(over: Partial<VisionNavTelemetry>): HTMLElement {
    render(
      <NavigationTab ctx={harness().ctx} firmware="ardupilot" telemetryOverride={mkTelemetry(over)} />,
    );
    return within(screen.getByTestId("vn-sensors-rangefinder")).getByTestId("vn-rangefinder-pill");
  }

  it("colours the rangefinder pill by its health, not by its wiring", () => {
    expect(
      rfPill({
        mode: "optical_flow",
        rangefinderTopology: "companion",
        flowDistanceM: null,
        rangefinderError: "Serial port not found",
      }).dataset.kind,
    ).toBe("error");
    cleanup();
    expect(
      rfPill({ mode: "optical_flow", rangefinderTopology: "fc", flowDistanceM: null }).dataset.kind,
    ).toBe("error");
    cleanup();
    expect(
      rfPill({ mode: "optical_flow_degraded", rangefinderTopology: "fc", flowDistanceM: null })
        .dataset.kind,
    ).toBe("warn");
    cleanup();
    const ok = rfPill({ mode: "optical_flow", rangefinderTopology: "companion", flowDistanceM: 1.2 });
    expect(ok.dataset.kind).toBe("ok");
    expect(screen.getByTestId("vn-sensors-rangefinder").textContent).toContain("Companion");
  });
});

describe("diagnostics", () => {
  it("shows the rangefinder error, failed sends and unwired inputs in words", () => {
    render(
      <NavigationTab
        ctx={harness().ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({
          rangefinderError: "Serial port /dev/ttyS9 not found",
          emitErrors: 42,
          degradedReasons: ["mavlink_subscribe_denied:RAW_IMU", "event_subscribe_denied:pose"],
        })}
      />,
    );
    const diag = screen.getByTestId("vn-diagnostics");
    expect(diag.textContent).toContain("Serial port /dev/ttyS9 not found");
    expect(diag.textContent).toContain("Failed MAVLink sends: 42");
    expect(diag.textContent).toContain("Cannot read RAW_IMU from the flight controller");
    expect(diag.textContent).toContain("Cannot read the pose topic");
    expect(diag.textContent).not.toContain("subscribe_denied");
  });

  it("hides the diagnostics row when nothing is wrong", () => {
    render(
      <NavigationTab
        ctx={harness().ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ emitErrors: 0, degradedReasons: [], rangefinderError: null })}
      />,
    );
    expect(screen.queryByTestId("vn-diagnostics")).toBeNull();
  });
});

describe("empty and refused feed", () => {
  it("says it is waiting before the first snapshot, not that the companion is down", () => {
    render(<NavigationTab ctx={harness().ctx} firmware="ardupilot" />);
    const card = screen.getByTestId("vn-pre-arm");
    expect(screen.getByTestId("vn-pre-arm-waiting").textContent).toBe(
      "Waiting for the vision-nav agent…",
    );
    expect(card.textContent).not.toContain("Vision companion is not running");
    expect(screen.queryByTestId("vn-pre-arm-blocked")).toBeNull();
    expect(screen.getByTestId("vn-companion-pill").textContent).not.toContain("Offline");
  });

  it("says navigation telemetry was not granted when the host refuses it", async () => {
    render(<NavigationTab ctx={harness({ denyTelemetry: true }).ctx} firmware="ardupilot" />);
    await act(async () => {
      await Promise.resolve();
    });
    expect(screen.getByTestId("vn-pre-arm-denied").textContent).toBe(
      "Mission Control did not grant navigation telemetry.",
    );
    expect(screen.queryByTestId("vn-pre-arm-blocked")).toBeNull();
  });

  it("passes the legacy flow gate at exactly 50, as the agent does", () => {
    render(
      <NavigationTab
        ctx={harness().ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ flowQuality: 50 })}
      />,
    );
    const row = screen
      .getByTestId("vn-pre-arm")
      .querySelector('[data-check-id="flow_quality"]') as HTMLElement;
    expect(row.dataset.state).toBe("ok");
  });
});

describe("first-run tour", () => {
  it("does not open on every visit when storage is unavailable", () => {
    const own = Object.getOwnPropertyDescriptor(window, "localStorage");
    Object.defineProperty(window, "localStorage", {
      configurable: true,
      get() {
        throw new DOMException("denied", "SecurityError");
      },
    });
    try {
      render(<NavigationTab ctx={harness().ctx} firmware="ardupilot" telemetryOverride={mkTelemetry()} />);
      expect(screen.queryByTestId("vn-tour-overlay")).toBeNull();
      fireEvent.click(screen.getByTestId("vn-tour-replay"));
      expect(screen.getByTestId("vn-tour-overlay")).toBeTruthy();
    } finally {
      if (own) Object.defineProperty(window, "localStorage", own);
      else delete (window as unknown as Record<string, unknown>).localStorage;
    }
  });

  it("is a dialog with focus on Next, Escape to skip, and 48 px buttons", () => {
    render(<NavigationTab ctx={harness().ctx} firmware="ardupilot" telemetryOverride={mkTelemetry()} />);
    const overlay = screen.getByTestId("vn-tour-overlay");
    const dialog = screen.getByRole("dialog");
    expect(dialog.getAttribute("aria-modal")).toBe("true");
    expect(dialog.getAttribute("aria-labelledby")).toBeTruthy();
    const next = screen.getByTestId("vn-tour-next");
    expect(document.activeElement).toBe(next);
    for (const id of ["vn-tour-replay", "vn-tour-skip", "vn-tour-next"]) {
      const b = screen.getByTestId(id);
      expect(b.style.minHeight, id).toBe("48px");
      expect(b.style.minWidth, id).toBe("48px");
    }
    fireEvent.keyDown(overlay, { key: "Escape" });
    expect(screen.queryByTestId("vn-tour-overlay")).toBeNull();
    expect(window.localStorage.getItem("vision-nav-tour-seen")).toBe("true");
  });
});

describe("engage state", () => {
  it("says vision navigation is disengaged when the engage state is idle", () => {
    const h = harness();
    render(<NavigationTab ctx={h.ctx} firmware="ardupilot" />);
    act(() => {
      h.emitTelemetry("navigation", {
        opticalFlowSupported: true,
        mode: "off",
        availableEstimators: ["off", "external_nav"],
        estimatorState: "off",
      });
      h.emitEvent("navigation.engage", { state: "idle" });
    });
    const note = screen.getByTestId("vn-mode-disengaged");
    expect(note.textContent).toContain("Disengaged");
    expect(note.textContent).toContain("Engage skill (Shift+N)");
    expect(note.textContent).toContain("Off is the effective mode");
    act(() => {
      h.emitEvent("navigation.engage", { state: "active" });
    });
    expect(screen.queryByTestId("vn-mode-disengaged")).toBeNull();
  });
});

describe("locale", () => {
  const PSEUDO = (key: string): string => `⟦${key}⟧`;
  // A word of three or more letters left after the locale markers are
  // removed is copy that did not go through the locale. Parameter names
  // (FLOW_TYPE, opflow_hardware) and units (Hz, ms) pass.
  const LOOSE_WORD = /(^|[^A-Za-z0-9_])([A-Za-z][a-z]{2,})(?![A-Za-z0-9_])/;

  function unrouted(): string[] {
    const out: string[] = [];
    const visit = (node: Node): void => {
      if (node.nodeType === 3) {
        const rest = (node.textContent ?? "").replace(/⟦[^⟧]*⟧/g, " ");
        if (LOOSE_WORD.test(rest)) out.push(node.textContent ?? "");
      }
      node.childNodes.forEach(visit);
    };
    visit(document.body);
    for (const el of Array.from(document.body.querySelectorAll("[title],[aria-label]"))) {
      for (const attr of ["title", "aria-label"]) {
        const v = el.getAttribute(attr);
        if (v && LOOSE_WORD.test(v.replace(/⟦[^⟧]*⟧/g, " "))) out.push(`${attr}=${v}`);
      }
    }
    return out;
  }

  const SCENARIOS: Array<() => JSX.Element> = [
    () => (
      <NavigationTab
        ctx={harness({ locale: PSEUDO }).ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({
          mode: "optical_flow",
          availableEstimators: ["off", "optical_flow", "optical_flow_degraded", "external_nav"],
          estimatorState: "degraded",
          imuSource: "mavlink-raw-imu",
          imuRateHz: 200,
          cameraImuSyncOffsetMs: 4,
          cameraIntrinsicsLoaded: true,
          rangefinderTopology: "companion",
          flowScaleSource: "baro",
          emitErrors: 3,
          degradedReasons: ["mavlink_subscribe_denied:RAW_IMU"],
        })}
        navStateOverride={normalizeNavState({ mode: "external_nav", active_set: 1, extnav_health: "good", odom_rate_hz: 10, gps_fix: 3, obstacle_rate_hz: 10, min_obstacle_m: 2, last_reset_counter: 0 })}
      />
    ),
    () => <NavigationTab ctx={harness({ locale: PSEUDO }).ctx} firmware="px4" telemetryOverride={mkTelemetry({ estimatorState: "failed", flowQuality: 10 })} />,
    () => <NavigationTab ctx={harness({ locale: PSEUDO }).ctx} firmware="inav" telemetryOverride={mkTelemetry({ companionState: "critical" })} />,
    () => <NavigationTab ctx={harness({ locale: PSEUDO }).ctx} firmware="betaflight" />,
    () => <NavigationTab ctx={harness({ locale: PSEUDO, denyTelemetry: true }).ctx} firmware="ardupilot" />,
  ];

  it("routes every operator string through the locale", async () => {
    for (const scenario of SCENARIOS) {
      render(scenario());
      await act(async () => {
        await Promise.resolve();
      });
      expect(unrouted()).toEqual([]);
      cleanup();
    }
  });

  it("ships every key the tab asks for, and no key the tab never uses", async () => {
    const requested = new Set<string>();
    for (const scenario of SCENARIOS) {
      const h = harness();
      const el = scenario();
      render(<NavigationTab {...el.props} ctx={h.ctx} />);
      await act(async () => {
        await Promise.resolve();
      });
      for (const k of h.requestedKeys) requested.add(k);
      cleanup();
    }
    expect([...requested].filter((k) => !EN_KEYS.has(k))).toEqual([]);

    const src = sourceText(SRC);
    const unused = [...EN_KEYS].filter((key) => {
      if (src.includes(`"${key}"`)) return false;
      // A key built from a template (`navigation.navState.health.${h}`),
      // or under a prefix handed to a shared card (`"navigation.params.px4"`).
      const parts = key.split(".");
      for (let i = parts.length - 1; i > 1; i--) {
        const prefix = parts.slice(0, i).join(".");
        if (src.includes(`${prefix}.\${`) || src.includes(`"${prefix}"`)) return false;
      }
      return true;
    });
    expect(unused).toEqual([]);
  });
});

describe("component source", () => {
  it("has no literal JSX text or literal title in any component", () => {
    const dir = resolve(SRC, "components");
    const hits: string[] = [];
    for (const name of readdirSync(dir).filter((n) => n.endsWith(".tsx"))) {
      const text = readFileSync(join(dir, name), "utf8");
      // Text between a tag's `>` and the next `<` or `{`, and string
      // literal title / aria-label attributes.
      for (const m of text.matchAll(/>\s*((?=[^<>{}=;()]*[A-Za-z]{3,})[A-Za-z][^<>{}=;()]*?)\s*[<{]/g)) {
        hits.push(`${name}: ${m[1]!.trim()}`);
      }
      for (const m of text.matchAll(/(?:title|aria-label)="([^"]+)"/g)) {
        hits.push(`${name}: ${m[1]}`);
      }
    }
    expect(hits).toEqual([]);
  });
});

describe("companion noun", () => {
  it("calls the companion pill the companion computer", () => {
    render(<NavigationTab ctx={harness().ctx} firmware="ardupilot" telemetryOverride={mkTelemetry()} />);
    expect(screen.getByTestId("vn-companion-pill").textContent).toBe("Companion computer: Active");
  });
});

describe("words", () => {
  it("names the IMU and scale sources in words", () => {
    render(
      <NavigationTab
        ctx={harness().ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({
          imuSource: "mavlink-raw-imu",
          flowScaleSource: "baro",
          rangefinderTopology: "companion",
        })}
      />,
    );
    const imu = screen.getByTestId("vn-sensors-imu").textContent ?? "";
    expect(imu).toContain("Flight controller IMU (RAW_IMU)");
    expect(imu).not.toContain("mavlink-raw-imu");
    const rf = screen.getByTestId("vn-sensors-rangefinder").textContent ?? "";
    expect(rf).toContain("Barometer");
    expect(rf).not.toMatch(/\bbaro\b/);
  });

  it("explains the fallback banner without shorthand", () => {
    render(
      <NavigationTab
        ctx={harness().ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ estimatorState: "degraded", flowScaleSource: null })}
      />,
    );
    expect(screen.getByTestId("vn-fallback-banner-degraded").textContent).not.toContain("OF scaling");
    cleanup();
    render(
      <NavigationTab
        ctx={harness().ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ estimatorState: "degraded", flowScaleSource: "baro", flowQuality: 20 })}
      />,
    );
    expect(screen.getByTestId("vn-fallback-banner-degraded").textContent).not.toContain("safe gate");
  });

  it("uses the shared nav words: Set n, Odometry Hz and the fix names", () => {
    const fix = (gps_fix: number): string => {
      render(
        <NavigationTab
          ctx={harness().ctx}
          firmware="ardupilot"
          telemetryOverride={mkTelemetry()}
          navStateOverride={normalizeNavState({ mode: "external_nav", active_set: 1, odom_rate_hz: 9.6, gps_fix })}
        />,
      );
      const text = screen.getByTestId("vn-nav-gps-fix").textContent ?? "";
      expect(screen.getByTestId("vn-nav-active-set").textContent).toBe("Set 1");
      expect(screen.getByTestId("vn-nav-odom-rate").textContent).toBe("10 Hz");
      expect(screen.getByTestId("vn-nav-state").textContent).toContain("Odometry");
      expect(screen.getByTestId("vn-nav-state").textContent).not.toContain("ODOMETRY");
      cleanup();
      return text;
    };
    expect(fix(0)).toBe("No GPS");
    expect(fix(3)).toBe("3D");
    expect(fix(6)).toBe("RTK fixed");
    expect(fix(7)).toBe("Static");
    expect(fix(8)).toBe("PPP");
    expect(fix(42)).toBe("Unknown");
  });
});

describe("mode list", () => {
  it("lists the modes as static items with the description inline", () => {
    render(
      <NavigationTab
        ctx={harness().ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({
          mode: "optical_flow",
          availableEstimators: ["off", "optical_flow", "optical_flow_degraded"],
        })}
      />,
    );
    const tile = screen.getByTestId("vn-mode-optical_flow_degraded");
    expect(tile.tagName).toBe("LI");
    expect(tile.style.cursor).not.toBe("pointer");
    expect(tile.textContent).toContain("baro / GPS (outdoor) ladder");
    const selected = screen.getByTestId("vn-mode-optical_flow");
    expect(selected.getAttribute("aria-current")).toBe("true");
    for (const el of Array.from(selected.querySelectorAll<HTMLElement>("*"))) {
      if (el.style.color) expect(el.style.color).toContain("--vn-text,");
    }
  });
});

describe("trends", () => {
  it("keeps a sample per heartbeat even when the value is steady", () => {
    const ctx = harness().ctx;
    const { rerender } = render(
      <NavigationTab ctx={ctx} firmware="ardupilot" telemetryOverride={mkTelemetry({ flowQuality: 186 })} />,
    );
    const spark = (): string => screen.getByTestId("vn-sparkline-flow-quality").textContent ?? "";
    expect(spark()).toContain("186");
    expect(spark().toLowerCase()).not.toContain("no data");
    rerender(<NavigationTab ctx={ctx} firmware="ardupilot" telemetryOverride={mkTelemetry({ flowQuality: 186 })} />);
    expect(screen.getByTestId("vn-sparkline-flow-quality").querySelector("polyline")).toBeTruthy();
  });
});

describe("ArduPilot parameter card", () => {
  it("lists every ArduPilot parameter the manifest requires", () => {
    const manifest = readFileSync(MANIFEST, "utf8");
    const block = manifest.split(/\n  ardupilot:\n/)[1]!.split(/\n  px4:\n/)[0]!;
    const params = [...block.matchAll(/param: "([A-Z0-9_]+)"/g)].map((m) => m[1]!);
    expect(params.length).toBeGreaterThan(10);
    render(<NavigationTab ctx={harness().ctx} firmware="ardupilot" telemetryOverride={mkTelemetry()} />);
    const card = screen.getByTestId("vn-ardupilot-params").textContent ?? "";
    for (const p of params) expect(card, p).toContain(p);
    expect(card).toContain("External navigation");
    expect(card).toContain("Obstacle feed");
  });
});
