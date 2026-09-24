import {
  HostError,
  PROTOCOL_VERSION,
  type RpcEnvelope,
  type TelemetryTopic,
} from "./protocol";
import { createWindowTransport, type Transport } from "./transport";

type EventHandler<TArgs = unknown> = (args: TArgs) => void;

/** Host event carrying the capability token to stamp on every request. */
export const CAPABILITY_TOKEN_EVENT = "capability.token";

/** One in-flight request awaiting the host's response. */
interface PendingRequest {
  resolve: (value: unknown) => void;
  reject: (err: HostError) => void;
  /** Cancels the request deadline once the request settles. */
  cancelDeadline: () => void;
}

/**
 * The PluginClient is the single public surface a plugin uses to
 * round-trip with the GCS host. It hides envelope assembly, capability
 * resolution, correlation IDs, and event routing.
 */
export class PluginClient {
  private readonly transport: Transport;
  private readonly pending = new Map<string, PendingRequest>();
  private readonly subscriptions = new Map<string, Set<EventHandler>>();
  private readonly disposers: Array<() => void> = [];
  private readonly idGen: () => string;
  /**
   * The latest capability token the host published on the
   * `capability.token` event. A host that verifies tokens rejects any
   * request without one, so every request carries the current value.
   */
  private token: string | null = null;

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
      pending.cancelDeadline();
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
      const slot: PendingRequest = {
        resolve: resolve as (value: unknown) => void,
        reject,
        cancelDeadline: () => {},
      };
      this.pending.set(id, slot);
      const env: RpcEnvelope<TArgs> = {
        id,
        type: "request",
        method,
        capability,
        args,
        version: PROTOCOL_VERSION,
        ...(this.token !== null ? { token: this.token } : {}),
      };
      this.transport.send(env);
      const timeoutMs = options?.timeoutMs ?? 5_000;
      if (timeoutMs > 0 && timeoutMs !== Number.POSITIVE_INFINITY) {
        const timer = setTimeout(() => {
          if (this.pending.get(id) !== slot) return;
          this.pending.delete(id);
          slot.reject(
            new HostError(
              "timeout",
              `host did not respond within ${timeoutMs}ms (method=${method})`,
            ),
          );
        }, timeoutMs);
        slot.cancelDeadline = () => clearTimeout(timer);
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

  /**
   * Subscribe to one telemetry topic. Resolves once the host accepted the
   * subscription, with a function that drops the handler and, when it was the
   * last handler for the topic on this client, stops the host's stream too.
   * A refused subscription leaves no handler behind.
   */
  async subscribeTelemetry<TArgs = unknown>(
    topic: TelemetryTopic | string,
    handler: EventHandler<TArgs>,
  ): Promise<() => void> {
    const eventMethod = `telemetry.${topic}`;
    return this.openStream(eventMethod, handler, {
      open: () =>
        this.request(
          "telemetry.subscribe",
          `telemetry.subscribe.${topic}`,
          { topic },
        ),
      close: () => this.request("telemetry.unsubscribe", "", { topic }),
    });
  }

  /**
   * Subscribe to the host's perception detection stream. Mirrors
   * {@link subscribeTelemetry}: it registers a local handler for the
   * pushed `perception.detections` event, then sends one
   * `perception.subscribe` request to open the stream. The returned
   * function tears the subscription down locally AND, for the last local
   * handler, sends a `perception.unsubscribe` request so the host stops
   * streaming.
   */
  async subscribePerception<TBatch = unknown>(
    handler: EventHandler<TBatch>,
  ): Promise<() => void> {
    return this.openStream("perception.detections", handler, {
      open: () =>
        this.request("perception.subscribe", "perception.subscribe", {}),
      close: () =>
        this.request("perception.unsubscribe", "perception.subscribe", {}),
    });
  }

  /**
   * Register `handler` for a host-pushed stream, then open the stream on the
   * host. The close request is best-effort (fire-and-forget, host errors
   * swallowed) so unmount paths never throw.
   */
  private async openStream<TArgs>(
    eventMethod: string,
    handler: EventHandler<TArgs>,
    stream: { open: () => Promise<unknown>; close: () => Promise<unknown> },
  ): Promise<() => void> {
    const off = this.on(eventMethod, handler);
    try {
      await stream.open();
    } catch (err) {
      off();
      throw err;
    }
    let closed = false;
    return () => {
      if (closed) return;
      closed = true;
      off();
      if (this.subscriptions.has(eventMethod)) return;
      void stream.close().catch(() => {});
    };
  }

  private route(env: RpcEnvelope): void {
    if (env.type === "response") {
      const slot = this.pending.get(env.id);
      if (!slot) return;
      this.pending.delete(env.id);
      slot.cancelDeadline();
      if (env.error) {
        slot.reject(new HostError(env.error.code, env.error.message));
      } else {
        slot.resolve(env.args);
      }
      return;
    }
    if (env.type === "event") {
      if (env.method === CAPABILITY_TOKEN_EVENT) {
        const token = (env.args as { token?: unknown } | null)?.token;
        this.token = typeof token === "string" && token.length > 0 ? token : null;
      }
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
