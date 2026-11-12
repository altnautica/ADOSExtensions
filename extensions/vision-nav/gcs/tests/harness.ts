import { HostError, MemoryTransport, PluginClient, type PluginContext } from "@altnautica/plugin-sdk";

export type Locale = Record<string, string> | ((key: string) => string);

export interface Harness {
  ctx: PluginContext;
  /** Push a payload on a `ctx.telemetry.subscribe` topic. */
  emitTelemetry(topic: string, payload: unknown): void;
  /** Push a payload on a `ctx.events.subscribe` topic. */
  emitEvent(topic: string, payload: unknown): void;
  /** Every key the tab asked the host to translate. */
  requestedKeys: Set<string>;
}

export interface HarnessOptions {
  locale?: Locale;
  /** The host refuses `telemetry.subscribe` with `capability_denied`. */
  denyTelemetry?: boolean;
  /** `telemetry.subscribe` fails with this host error code (not a denial). */
  failTelemetry?: string;
}

/** A plugin context whose telemetry and event subscriptions the test drives. */
export function harness(opts: HarnessOptions = {}): Harness {
  const telemetry = new Map<string, Array<(args: unknown) => void>>();
  const events = new Map<string, Array<(args: unknown) => void>>();
  const requestedKeys = new Set<string>();
  const locale = opts.locale ?? {};
  const add = (
    m: Map<string, Array<(args: unknown) => void>>,
    topic: string,
    h: (args: unknown) => void,
  ): (() => void) => {
    const list = m.get(topic) ?? [];
    list.push(h);
    m.set(topic, list);
    return () => {
      m.set(
        topic,
        (m.get(topic) ?? []).filter((x) => x !== h),
      );
    };
  };
  const ctx = {
    client: new PluginClient({ transport: new MemoryTransport() }),
    telemetry: {
      subscribe: (topic: string, h: (args: unknown) => void) => {
        if (opts.denyTelemetry) {
          return Promise.reject(new HostError("capability_denied", "refused"));
        }
        if (opts.failTelemetry) {
          return Promise.reject(new HostError(opts.failTelemetry, "failed"));
        }
        return Promise.resolve(add(telemetry, topic, h));
      },
    },
    command: { send: async () => ({ ok: true }) },
    notifications: { publish: async () => ({ ok: true }) },
    recording: { mark: async () => ({ ok: true }) },
    mission: { read: async () => ({}), write: async () => ({ ok: true }) },
    perception: {
      readTier: async () => ({ tier: null, offloadTarget: null }),
      subscribeDetections: async () => () => undefined,
      readSessionHealth: async () => ({
        session: "closed" as const,
        feed: "idle" as const,
        ageMs: null,
        batchesPerSecond: null,
        boundNode: null,
      }),
    },
    events: {
      subscribe: (topic: string, h: (args: unknown) => void) => add(events, topic, h),
      listen: async () => () => undefined,
      publish: async () => undefined,
    },
    records: {
      list: async () => [],
      get: async () => null,
      put: async () => undefined,
      remove: async () => undefined,
    },
    config: { onChange: () => () => undefined },
    theme: { onChange: () => () => undefined },
    videoOverlay: { onProps: () => () => undefined },
    i18n: {
      t: (key: string) => {
        requestedKeys.add(key);
        return typeof locale === "function" ? locale(key) : (locale[key] ?? key);
      },
    },
  } as unknown as PluginContext;
  return {
    ctx,
    requestedKeys,
    emitTelemetry: (topic, payload) => {
      for (const h of telemetry.get(topic) ?? []) h(payload);
    },
    emitEvent: (topic, payload) => {
      for (const h of events.get(topic) ?? []) h(payload);
    },
  };
}
