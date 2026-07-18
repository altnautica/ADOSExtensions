import { describe, expect, it } from "vitest";

import { PodStateStore } from "../src/pod-state";

describe("PodStateStore", () => {
  it("ingests a payload and notifies subscribers", () => {
    const store = new PodStateStore();
    const seen: string[] = [];
    store.subscribe((s) => seen.push(s.model));
    store.ingest({ model: "ZT30", connected: true, known: true });
    expect(store.get()?.model).toBe("ZT30");
    expect(store.get()?.connected).toBe(true);
    expect(seen).toContain("ZT30");
  });

  it("replays the latest state to a late subscriber", () => {
    const store = new PodStateStore();
    store.ingest({ model: "A8 mini" });
    let got: string | null = null;
    store.subscribe((s) => (got = s.model));
    expect(got).toBe("A8 mini");
  });

  it("normalises missing and malformed fields to safe defaults", () => {
    const store = new PodStateStore();
    store.ingest({ model: "ZR30", laser_range_m: "oops", zoom: 5.5 });
    const s = store.get();
    expect(s?.laser_range_m).toBeNull();
    expect(s?.zoom).toBe(5.5);
    expect(s?.recording).toBe(false);
  });

  it("ignores non-object payloads", () => {
    const store = new PodStateStore();
    store.ingest(null);
    store.ingest(42);
    expect(store.get()).toBeNull();
  });

  it("unsubscribe stops notifications", () => {
    const store = new PodStateStore();
    let count = 0;
    const unsub = store.subscribe(() => count++);
    store.ingest({ model: "a" });
    unsub();
    store.ingest({ model: "b" });
    expect(count).toBe(1);
  });
});
