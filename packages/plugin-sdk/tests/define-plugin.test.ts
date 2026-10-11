import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { createPluginContext } from "../src/api";
import { PluginClient } from "../src/client";
import { definePlugin } from "../src/definePlugin";
import { MemoryTransport } from "../src/transport";

// A frame's window: the plugin listens on it, the host is its `parent`.
const hostWindow = { name: "host" };
let frame: EventTarget & { parent: unknown };

function postFromHost(data: unknown, source: unknown = hostWindow): void {
  const ev = new Event("message");
  Object.defineProperty(ev, "data", { value: data });
  Object.defineProperty(ev, "source", { value: source });
  frame.dispatchEvent(ev);
}

function memoryContext() {
  return createPluginContext({
    client: new PluginClient({ transport: new MemoryTransport() }),
  });
}

async function settle(): Promise<void> {
  for (let i = 0; i < 5; i += 1) await Promise.resolve();
}

beforeEach(() => {
  frame = Object.assign(new EventTarget(), { parent: hostWindow });
  vi.stubGlobal("window", frame);
});

afterEach(() => {
  vi.unstubAllGlobals();
});

describe("definePlugin host unmount message", () => {
  it("runs the plugin's unmount when the host posts { type: 'unmount' }", async () => {
    const unmount = vi.fn();
    const ctx = memoryContext();
    const dispose = vi.spyOn(ctx.client, "dispose");
    definePlugin({ id: "com.example.p", version: "1.0.0", context: ctx, mount: () => {}, unmount });
    await settle();

    postFromHost({ type: "unmount" });
    await settle();

    expect(unmount).toHaveBeenCalledTimes(1);
    expect(unmount).toHaveBeenCalledWith(ctx, { id: "com.example.p", version: "1.0.0" });
    expect(dispose).toHaveBeenCalledTimes(1);
  });

  it("waits for a mount still in flight, then unmounts once", async () => {
    const order: string[] = [];
    let finishMount: () => void = () => {};
    definePlugin({
      id: "com.example.p",
      version: "1.0.0",
      context: memoryContext(),
      mount: () =>
        new Promise<void>((resolve) => {
          finishMount = () => {
            order.push("mounted");
            resolve();
          };
        }),
      unmount: () => {
        order.push("unmounted");
      },
    });

    postFromHost({ type: "unmount" });
    postFromHost({ type: "unmount" });
    await settle();
    expect(order).toEqual([]);

    finishMount();
    await settle();
    expect(order).toEqual(["mounted", "unmounted"]);
  });

  it("ignores the message from a source other than the host, and other types", async () => {
    const unmount = vi.fn();
    definePlugin({ id: "com.example.p", version: "1.0.0", context: memoryContext(), mount: () => {}, unmount });
    await settle();

    postFromHost({ type: "unmount" }, { name: "other" });
    postFromHost({ type: "mount" });
    postFromHost("unmount");
    postFromHost(null);
    await settle();

    expect(unmount).not.toHaveBeenCalled();
  });

  it("stops listening after an explicit unmount", async () => {
    const unmount = vi.fn();
    const plugin = definePlugin({
      id: "com.example.p",
      version: "1.0.0",
      context: memoryContext(),
      mount: () => {},
      unmount,
    });
    await settle();

    await plugin.unmount();
    postFromHost({ type: "unmount" });
    await settle();

    expect(unmount).toHaveBeenCalledTimes(1);
  });
});
