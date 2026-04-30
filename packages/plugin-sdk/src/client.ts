import {
  HostError,
  PROTOCOL_VERSION,
  type RpcEnvelope,
  type TelemetryTopic,
} from "./protocol";
import { createWindowTransport, type Transport } from "./transport";

type EventHandler<TArgs = unknown> = (args: TArgs) => void;

/**
 * The PluginClient is the single public surface a plugin uses to
 * round-trip with the GCS host. It hides envelope assembly, capability
 * resolution, correlation IDs, and event routing.
 */
export class PluginClient {
  private readonly transport: Transport;
  private readonly pending = new Map<
    string,
    {
      resolve: (value: unknown) => void;
      reject: (err: HostError) => void;
    }
  >();
  private readonly subscriptions = new Map<string, Set<EventHandler>>();
  private readonly disposers: Array<() => void> = [];
  private readonly idGen: () => string;

  constructor(opts?: { transport?: Transport; idGen?: () => string }) {
    this.transport = opts?.transport ?? createWindowTransport();
    this.idGen = opts?.idGen ?? defaultIdGen;
    this.disposers.push(this.transport.onMessage((env) => this.route(env)));
  }

  /** Tear down listeners and reject any in-flight RPCs. */
  dispose(): void {
    for (const d of this.disposers) d();
    this.disposers.length = 0;
    for (const [, pending] of this.pending) {
      pending.reject(new HostError("disposed", "client disposed"));
    }
    this.pending.clear();
    this.subscriptions.clear();
  }

  /** Send a request and wait for the host's response. */
  request<TResult = unknown, TArgs = unknown>(
    method: string,
    capability: string,
    args: TArgs,
    options?: { timeoutMs?: number },
  ): Promise<TResult> {
    const id = this.idGen();
    return new Promise<TResult>((resolve, reject) => {
      this.pending.set(id, {
        resolve: resolve as (value: unknown) => void,
        reject,
      });
      const env: RpcEnvelope<TArgs> = {
        id,
        type: "request",
        method,
        capability,
        args,
        version: PROTOCOL_VERSION,
      };
      this.transport.send(env);
      const timeoutMs = options?.timeoutMs ?? 5_000;
      if (timeoutMs > 0 && timeoutMs !== Number.POSITIVE_INFINITY) {
        setTimeout(() => {
          const slot = this.pending.get(id);
          if (!slot) return;
          this.pending.delete(id);
          slot.reject(
            new HostError(
              "timeout",
              `host did not respond within ${timeoutMs}ms (method=${method})`,
            ),
          );
        }, timeoutMs);
      }
    });
  }

  /**
   * Subscribe to host-pushed events for one method (e.g. theme.changed,
   * telemetry.battery). Returns an unsubscribe function.
   */
  on<TArgs = unknown>(
    method: string,
    handler: EventHandler<TArgs>,
  ): () => void {
    const set = this.subscriptions.get(method) ?? new Set<EventHandler>();
    set.add(handler as EventHandler);
    this.subscriptions.set(method, set);
    return () => {
      const live = this.subscriptions.get(method);
      if (!live) return;
      live.delete(handler as EventHandler);
      if (live.size === 0) this.subscriptions.delete(method);
    };
  }

  /** Convenience for the most common subscription pattern. */
  async subscribeTelemetry<TArgs = unknown>(
    topic: TelemetryTopic | string,
    handler: EventHandler<TArgs>,
  ): Promise<() => void> {
    const eventMethod = `telemetry.${topic}`;
    const off = this.on(eventMethod, handler);
    await this.request(
      "telemetry.subscribe",
      `telemetry.subscribe.${topic}`,
      { topic },
    );
    return off;
  }

  private route(env: RpcEnvelope): void {
    if (env.type === "response") {
      const slot = this.pending.get(env.id);
      if (!slot) return;
      this.pending.delete(env.id);
      if (env.error) {
        slot.reject(new HostError(env.error.code, env.error.message));
      } else {
        slot.resolve(env.args);
      }
      return;
    }
    if (env.type === "event") {
      const set = this.subscriptions.get(env.method);
      if (!set) return;
      for (const fn of set) fn(env.args);
    }
  }
}

function defaultIdGen(): string {
  if (typeof crypto !== "undefined" && "randomUUID" in crypto) {
    return crypto.randomUUID();
  }
  return `id-${Date.now()}-${Math.random().toString(36).slice(2)}`;
}
