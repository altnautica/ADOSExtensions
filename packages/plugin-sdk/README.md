# @altnautica/plugin-sdk

TypeScript SDK for ADOS Mission Control plugins. Wraps the postMessage
RPC envelope, the capability-gated transport, and a typed
`PluginContext` so plugin authors never touch the wire shape directly.

## Install

```sh
pnpm add @altnautica/plugin-sdk
# or
npm install @altnautica/plugin-sdk
```

## Hello plugin

```ts
import { definePlugin } from "@altnautica/plugin-sdk";

definePlugin({
  id: "com.example.hello",
  version: "1.0.0",
  async mount(ctx) {
    await ctx.notifications.publish({
      channelId: "hello",
      severity: "info",
      title: "Hello from a plugin",
    });
  },
});
```

## Subscribing to telemetry

```ts
await ctx.telemetry.subscribe<BatterySample>("battery", (sample) => {
  // ...
});
```

The capability id is derived from the topic: `telemetry.subscribe.<topic>`.
The plugin must declare matching `permissions` in its manifest.

## The `contributes` model

A plugin declares what it adds to the GCS in its manifest's
`gcs.contributes` block. The SDK ships typed shapes for every
contribution kind so you can author the YAML against a compile-time
check. The `contributesModel()` helper is an identity function that
gives you editor autocomplete while building the object (e.g. in a test
fixture or a codegen step):

```ts
import { contributesModel } from "@altnautica/plugin-sdk";

const contributes = contributesModel({
  // A native, declarative settings panel — no iframe.
  parameters: [
    {
      key: "follow_distance_m",
      schema: { type: "number", minimum: 3, maximum: 30, default: 8 },
      ui: { widget: "range", label: "settings.followDistance" },
    },
    {
      // A model picker bound to the shared vision detector.
      key: "detector",
      schema: { type: "string" },
      binding: "engine.detector",
      ui: { widget: "model", task: "detection" },
    },
  ],
  // A detail tab the host mounts on a node's detail panel.
  tabs: [{ id: "settings", profile: ["drone"], title: "settings.tab" }],
  // A vision model the picker can select.
  models: [
    {
      id: "coco-person",
      task: "detection",
      board_variants: [{ board_match: "rk3588", runtime: "rknn", min_tops: 6 }],
    },
  ],
});
```

Contribution kinds: `skills` (cockpit Skill Bar), `panels` / `overlays`
/ `notifications` (iframe slots), `parameters` (native declarative
controls), `tabs` (the `node.detail.tab` slot, optionally narrowed by
node `profile`), `settings` (sections of native parameters), `models`
(vision-model registrations), `missionTemplates`, and `mapOverlays`.

A `parameters[]` entry's value is written to its `binding`:
`plugin.config` (default, the per-node config the agent half reads
live), `engine.detector` (the shared vision detector), or `agent.config`
(a whitelisted system key). Validate and clamp values against the same
rules the host applies on commit:

```ts
import { validateValue, clampValue, defaultFor } from "@altnautica/plugin-sdk";

const schema = { type: "number", minimum: 3, maximum: 30, step: 0.5 } as const;
validateValue(schema, 8); // { ok: true }
clampValue(schema, 99); // 30
defaultFor(schema); // 3 (no declared default -> the minimum)
```

## Testing

```ts
import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

const harness = createPluginHarness({
  grantedCapabilities: ["telemetry.subscribe.battery"],
  mount: async (ctx) => {
    await ctx.telemetry.subscribe("battery", (s) => store.ingest(s));
  },
});

await harness.start();
harness.pushTelemetry("battery", mockSample);
expect(harness.notifications).toHaveLength(1);
await harness.teardown();
```

The harness is a synthetic host: it captures every RPC the plugin
issues and lets you inject telemetry, config changes, theme updates,
and host failures.

## License

GPL-3.0-or-later.
