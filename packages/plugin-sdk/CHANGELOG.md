# Changelog

All notable changes to `@altnautica/plugin-sdk`.

## 1.5.0

- `ctx.config.onChange` and `ctx.theme.onChange` hand a handler registered
  after the host's push the last config or theme it sent. A plugin whose
  effects subscribe after the host posted the config on load used to miss
  it and show defaults until the next change. The replay runs on a
  microtask after the subscribe call, and is skipped when the handler
  unsubscribed or a newer push already reached it. Telemetry, perception
  and video overlay props are live samples and are never replayed.

## 1.4.0

- `ctx.videoOverlay.onProps(handler)` delivers the `video.overlay` host props
  (`video.overlay.props`): camera, stream size, the letterbox-corrected
  rendered rect, frame time and age, attitude (null when unknown) and the
  fresh detection batch (null once stale), typed as `VideoOverlayHostProps`.
  A payload without a rendered rect is dropped. Mission Control and the agent
  cockpit push the same shape.
- The harness adds `pushVideoOverlayProps(props)`.

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
