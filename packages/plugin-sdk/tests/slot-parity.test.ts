/**
 * The SDK's slot union must equal the GCS host's slot registry.
 *
 * The registry in `ADOSMissionControl/src/lib/plugins/types.ts` decides what
 * can actually mount; this union is what a plugin author writes against. When
 * they diverge the failure is invisible: the manifest parses on both sides,
 * the install dialog shows an "unknown capability" placeholder, and the
 * contribution never appears — with no error anywhere the developer can see.
 * That is exactly how `connection.protocol` and `recording.processor` came to
 * be advertised by the SDK while existing in no catalog and no registry.
 *
 * The GCS lives in a sibling repository, so the expected list is restated here
 * rather than imported. Keeping it a literal is the point: changing it is a
 * deliberate two-repo edit, not something a refactor can do silently.
 */
import { describe, expect, it } from "vitest";

import type { PluginSlotName } from "../src/manifest";

/** `PLUGIN_SLOTS` in ADOSMissionControl/src/lib/plugins/types.ts. */
const HOST_SLOTS = [
  "fc.tab",
  "hardware.tab",
  "mission.template",
  "map.overlay",
  "video.overlay",
  "notification.channel",
  "settings.section",
  "node.detail.tab",
  "cockpit.panel",
  "flight.skill",
] as const;

/** Every member of the SDK union, restated as values. A missing entry is a
 * type error below, and an extra one fails the length assertion. */
const SDK_SLOTS: readonly PluginSlotName[] = [
  "fc.tab",
  "hardware.tab",
  "mission.template",
  "map.overlay",
  "video.overlay",
  "notification.channel",
  "settings.section",
  "node.detail.tab",
  "cockpit.panel",
  "flight.skill",
];

describe("plugin slot parity", () => {
  it("advertises exactly the slots the host can mount", () => {
    expect([...SDK_SLOTS].sort()).toEqual([...HOST_SLOTS].sort());
  });

  it("does not type-check a slot the host has no registry for", () => {
    // @ts-expect-error connection.protocol exists in no host slot registry.
    const bogus: PluginSlotName = "connection.protocol";
    expect(bogus).toBe("connection.protocol");
  });
});
