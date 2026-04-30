import { createPluginContext, type PluginContext } from "./api";
import { PluginClient } from "./client";
import { PROTOCOL_VERSION, type RpcEnvelope } from "./protocol";
import { MemoryTransport } from "./transport";

/**
 * Test harness for plugin authors. Runs the plugin against an
 * in-memory transport so unit tests can:
 *
 *   - subscribe to telemetry topics and inject mock samples
 *   - assert on the sequence of RPC calls the plugin made
 *   - simulate config changes, theme changes, and capability denials
 *   - tear down deterministically
 *
 * Usage:
 *   const harness = createPluginHarness({
 *     mount: async (ctx) => { await ctx.telemetry.subscribe('battery', ...); },
 *   });
 *   await harness.start();
 *   harness.pushTelemetry('battery', sample);
 *   expect(harness.calls).toContainEqual({ method: 'notification.publish', ... });
 *   await harness.teardown();
 */

export interface PluginHarnessCall {
  method: string;
  capability: string;
  args: unknown;
}

export interface PluginHarnessOptions {
  id?: string;
  version?: string;
  locale?: Record<string, string>;
  /** Plugin mount function under test. */
  mount: (ctx: PluginContext) => Promise<void> | void;
  /** Optional unmount counterpart. */
  unmount?: (ctx: PluginContext) => Promise<void> | void;
  /**
   * Capabilities the simulated host has granted. The harness rejects
   * RPCs whose required capability isn't in this set, mirroring how
   * the real host bridge gates calls.
   */
  grantedCapabilities?: ReadonlyArray<string>;
  /**
   * Custom response generator. Returns the result the host should
   * send back. By default the harness echoes `{ ok: true }`.
   */
  respondTo?: (call: PluginHarnessCall) => Promise<unknown> | unknown;
}

export interface PluginHarness {
  ctx: PluginContext;
  /** Every RPC the plugin has issued during the test. */
  readonly calls: ReadonlyArray<PluginHarnessCall>;
  /** Notifications the plugin published, oldest-first. */
  readonly notifications: ReadonlyArray<unknown>;
  /** Recording markers the plugin wrote, oldest-first. */
  readonly recordingMarks: ReadonlyArray<unknown>;
  start(): Promise<void>;
  pushTelemetry(topic: string, payload: unknown): void;
  pushEvent(method: string, args: unknown, capability?: string): void;
  pushConfig(next: unknown): void;
  pushTheme(vars: Record<string, string>): void;
  /**
   * Simulate the host denying a future RPC by code. Subsequent matching
   * calls receive the configured error envelope.
   */
  failNext(method: string, code: string, message: string): void;
  teardown(): Promise<void>;
}

export function createPluginHarness(
  opts: PluginHarnessOptions,
): PluginHarness {
  const transport = new MemoryTransport();
  const granted = new Set(opts.grantedCapabilities ?? []);
  const calls: PluginHarnessCall[] = [];
  const notifications: unknown[] = [];
  const recordingMarks: unknown[] = [];
  const failQueue = new Map<string, { code: string; message: string }>();

  const respond = opts.respondTo ?? (() => ({ ok: true }));

  transport.onPluginSend = (env) => {
    if (env.type !== "request") return;
    calls.push({
      method: env.method,
      capability: env.capability,
      args: env.args,
    });
    if (env.method === "notification.publish") notifications.push(env.args);
    if (env.method === "recording.mark") recordingMarks.push(env.args);

    void Promise.resolve().then(async () => {
      const planned = failQueue.get(env.method);
      if (planned) {
        failQueue.delete(env.method);
        sendResponse(env, undefined, planned);
        return;
      }
      // Capability gate. The harness mirrors the real host: an RPC
      // whose declared capability isn't granted is rejected.
      if (
        env.capability &&
        !granted.has(env.capability) &&
        !env.capability.startsWith("ui.slot.") &&
        env.method !== "telemetry.subscribe"
      ) {
        sendResponse(env, undefined, {
          code: "permission_denied",
          message: `plugin lacks capability ${env.capability}`,
        });
        return;
      }
      const result = await respond({
        method: env.method,
        capability: env.capability,
        args: env.args,
      });
      sendResponse(env, result);
    });
  };

  const client = new PluginClient({ transport });
  const ctx = createPluginContext({ client, locale: opts.locale });

  function sendResponse(
    request: RpcEnvelope,
    args?: unknown,
    error?: { code: string; message: string },
  ): void {
    transport.pushFromHost({
      id: request.id,
      type: "response",
      method: request.method,
      capability: request.capability,
      args: args ?? null,
      version: PROTOCOL_VERSION,
      error,
    });
  }

  return {
    ctx,
    get calls() {
      return calls;
    },
    get notifications() {
      return notifications;
    },
    get recordingMarks() {
      return recordingMarks;
    },
    async start() {
      await opts.mount(ctx);
    },
    pushTelemetry(topic, payload) {
      transport.pushFromHost({
        id: `telemetry-${topic}-${Date.now()}`,
        type: "event",
        method: `telemetry.${topic}`,
        capability: `telemetry.subscribe.${topic}`,
        args: payload,
        version: PROTOCOL_VERSION,
      });
    },
    pushEvent(method, args, capability = "") {
      transport.pushFromHost({
        id: `evt-${method}-${Date.now()}`,
        type: "event",
        method,
        capability,
        args,
        version: PROTOCOL_VERSION,
      });
    },
    pushConfig(next) {
      transport.pushFromHost({
        id: `config-${Date.now()}`,
        type: "event",
        method: "config.changed",
        capability: "",
        args: next,
        version: PROTOCOL_VERSION,
      });
    },
    pushTheme(vars) {
      transport.pushFromHost({
        id: `theme-${Date.now()}`,
        type: "event",
        method: "theme.changed",
        capability: "theme.useTheme",
        args: vars,
        version: PROTOCOL_VERSION,
      });
    },
    failNext(method, code, message) {
      failQueue.set(method, { code, message });
    },
    async teardown() {
      if (opts.unmount) await opts.unmount(ctx);
      ctx.client.dispose();
    },
  };
}
