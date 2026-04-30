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
