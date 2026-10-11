# Changelog

All notable changes to `@altnautica/plugin-sdk`.

## 1.3.0

- `definePlugin` runs the plugin's `unmount` hook when the host posts
  `{ type: "unmount" }` into the frame before tearing it down. The host keeps
  the document alive for up to 500 ms for this. A mount still in flight
  finishes first, the hook runs once, and messages from any window other than
  the host are ignored.
- The test harness gates a telemetry subscription on its full topic
  capability, `telemetry.subscribe.<topic>`, the way the host resolves it. It
  used to let every subscription through, so a test could not catch a manifest
  missing the topic grant. A test that subscribes now lists the topic in
  `grantedCapabilities`.
- The harness gates `ui.slot.*` requests like any other capability. Only
  host-pushed events, which never pass the request gate, carry a slot
  capability without a grant.
- `PluginSlotName` adds `cockpit.widget`, `node.agent.page` and
  `node.surface`, matching the host's slot registry.
- `PluginClient.request` types its capability argument as
  `RequestCapability`: an id from the generated capability catalog, a
  `telemetry.subscribe.<topic>` grant, a `plugin.<...>` scoped id, or `""`.
  A misspelled capability is a type error instead of a runtime denial.
