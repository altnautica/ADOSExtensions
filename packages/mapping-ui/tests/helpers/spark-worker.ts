/**
 * An in-process `Worker` for Spark's splat worker under Node. Spark bundles
 * its worker (the Rust/wasm decoders) as source and starts it from a `data:`
 * URL when no blob URL is available; this shim evaluates that source with a
 * `self` whose messages are delivered asynchronously, so `SplatMesh` decodes
 * files in a test exactly as it does in the browser.
 */

type Listener = (event: { data: unknown }) => void;

class Port {
  private readonly listeners = new Set<Listener>();
  onmessage: Listener | null = null;
  constructor(private readonly peer: () => Port) {}
  addEventListener(type: string, l: Listener): void {
    if (type === "message") this.listeners.add(l);
  }
  removeEventListener(type: string, l: Listener): void {
    if (type === "message") this.listeners.delete(l);
  }
  postMessage(data: unknown): void {
    const to = this.peer();
    setTimeout(() => to.deliver(data), 0);
  }
  deliver(data: unknown): void {
    const event = { data };
    this.onmessage?.(event);
    for (const l of this.listeners) l(event);
  }
}

const DATA_JS = "data:text/javascript;charset=utf-8,";

export class InProcessWorker extends Port {
  private readonly inner: Port;
  constructor(url: string | URL) {
    const href = String(url);
    if (!href.startsWith(DATA_JS)) throw new Error(`only ${DATA_JS} workers run in process`);
    let inner: Port | null = null;
    super(() => inner as Port);
    const outer = this as Port;
    inner = new Port(() => outer);
    this.inner = inner;
    new Function("self", decodeURIComponent(href.slice(DATA_JS.length)))(inner);
  }
  terminate(): void {
    this.inner.onmessage = null;
  }
}

/** Install the shim as the global `Worker`; returns the restore function. */
export function installInProcessWorker(): () => void {
  const g = globalThis as { Worker?: unknown };
  const prev = g.Worker;
  g.Worker = InProcessWorker;
  return () => {
    g.Worker = prev;
  };
}
