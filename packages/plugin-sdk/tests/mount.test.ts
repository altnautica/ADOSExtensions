import { describe, it, expect } from "vitest";

import { parseMount } from "../src/api";

describe("parseMount", () => {
  it("reads the slot and panel the host put in the fragment", () => {
    expect(parseMount("#slot=agent.page&panel=navigation")).toEqual({
      slot: "agent.page",
      panelId: "navigation",
    });
  });

  it("decodes URL-encoded values", () => {
    expect(parseMount("#slot=cockpit.panel&panel=a%20b%26c")).toEqual({
      slot: "cockpit.panel",
      panelId: "a b&c",
    });
  });

  it("leaves a missing field null", () => {
    expect(parseMount("#panel=navigation-health")).toEqual({
      slot: null,
      panelId: "navigation-health",
    });
  });

  it("names nothing for a host that passes no fragment", () => {
    expect(parseMount("")).toEqual({ slot: null, panelId: null });
  });
});
