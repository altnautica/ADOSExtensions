# Changelog

All notable changes to `@altnautica/plugin-sdk`.

## 1.5.3

- `SampleClock` ignores a sample stamped earlier than the one before it (an
  older report delivered late, as when a host falls back to a slower source
  after a failed poll): it returns `null`, so the panel keeps the newer
  reading and that reading still goes stale on time. Only a sample whose
  stamp moved forward continues a live run. The stamp is compared with the
  previous delivery only, so a drone clock that steps back costs one sample.

## 1.5.2

- `SampleClock`, `STALE_AFTER_MS` (2000) and `agentTimestampOf` age host
  state samples one way for every panel. The drone's `ts_ms` judges only the
  first sample of a run, so an old entry delivered on mount reads stale;
  later samples age from their arrival, so a live feed stays live when the
  drone clock runs behind or a poll is slow. A repeated report is ignored,
  and a feed that stops reads stale 2000 ms after its last sample.

## 1.5.1

- The host's camera list (`cameras.changed`, `{cameras: [{id, name}]}`) is
  handed to a late subscriber the same way as the config and theme, so a
  panel that subscribes after the host posted it still names its cameras.

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
