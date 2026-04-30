import {
  HostError,
  PROTOCOL_VERSION,
  type RpcEnvelope,
} from "./protocol";

/**
 * Abstracts the concrete postMessage channel so the SDK runs unchanged
 * inside a real iframe (default `WindowTransport`) or a unit test
 * (custom `MemoryTransport`). The host owns the other side of the
 * channel; this transport is the plugin half.
 */
export interface Transport {
  send(env: RpcEnvelope): void;
  /** Subscribe to envelopes coming from the host. */
  onMessage(handler: (env: RpcEnvelope) => void): () => void;
}

class IframeWindowTransport implements Transport {
  private readonly listeners = new Set<(env: RpcEnvelope) => void>();
  private bound = false;

  private readonly windowHandler = (ev: MessageEvent<RpcEnvelope>): void => {
    const data = ev.data;
    if (!data || typeof data !== "object" || data.version !== PROTOCOL_VERSION) {
      return;
    }
    for (const fn of this.listeners) fn(data);
  };

  private bindIfNeeded(): void {
    if (this.bound || typeof window === "undefined") return;
    window.addEventListener("message", this.windowHandler);
    this.bound = true;
  }

  send(env: RpcEnvelope): void {
    if (typeof window === "undefined" || !window.parent) {
      throw new HostError(
        "no_host",
        "no parent window. Plugin SDK requires a sandboxed iframe context.",
      );
    }
    window.parent.postMessage(env, "*");
  }

  onMessage(handler: (env: RpcEnvelope) => void): () => void {
    this.bindIfNeeded();
    this.listeners.add(handler);
    return () => {
      this.listeners.delete(handler);
      if (this.listeners.size === 0 && typeof window !== "undefined") {
        window.removeEventListener("message", this.windowHandler);
        this.bound = false;
      }
    };
  }
}

export function createWindowTransport(): Transport {
  return new IframeWindowTransport();
}

/** In-memory transport used by the test harness and unit tests. */
export class MemoryTransport implements Transport {
  private readonly handlers = new Set<(env: RpcEnvelope) => void>();
  /** Callback the test setup registers to observe what the plugin sent. */
  onPluginSend: ((env: RpcEnvelope) => void) | null = null;

  send(env: RpcEnvelope): void {
    this.onPluginSend?.(env);
  }

  onMessage(handler: (env: RpcEnvelope) => void): () => void {
    this.handlers.add(handler);
    return () => {
      this.handlers.delete(handler);
    };
  }

  /** Inject an envelope as if it had arrived from the host. */
  pushFromHost(env: RpcEnvelope): void {
    for (const fn of this.handlers) fn(env);
  }
}
