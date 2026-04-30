import { describe, it, expect } from "vitest";

import { PluginClient } from "../src/client";
import { HostError, PROTOCOL_VERSION } from "../src/protocol";
import { MemoryTransport } from "../src/transport";

function setup() {
  const transport = new MemoryTransport();
  let counter = 0;
  const client = new PluginClient({
    transport,
    idGen: () => `id-${++counter}`,
  });
  return { transport, client };
}

describe("PluginClient.request", () => {
  it("sends a v1 envelope and resolves with the response args", async () => {
    const { transport, client } = setup();
    transport.onPluginSend = (env) => {
      transport.pushFromHost({
        id: env.id,
        type: "response",
        method: env.method,
        capability: env.capability,
        args: { ok: true, echo: env.args },
        version: PROTOCOL_VERSION,
      });
    };
    const res = await client.request<{ ok: boolean; echo: unknown }>(
      "ping",
      "",
      { x: 1 },
    );
    expect(res.ok).toBe(true);
    expect(res.echo).toEqual({ x: 1 });
  });

  it("rejects with HostError carrying the code when the response has an error", async () => {
    const { transport, client } = setup();
    transport.onPluginSend = (env) => {
      transport.pushFromHost({
        id: env.id,
        type: "response",
        method: env.method,
        capability: env.capability,
        args: null,
        version: PROTOCOL_VERSION,
        error: { code: "permission_denied", message: "no" },
      });
    };
    await expect(
      client.request("vehicle.command", "vehicle.command", {}),
    ).rejects.toMatchObject({
      name: "HostError",
      code: "permission_denied",
    });
  });

  it("rejects with HostError(timeout) when the host never responds", async () => {
    const { client } = setup();
    await expect(
      client.request("ping", "", {}, { timeoutMs: 25 }),
    ).rejects.toMatchObject({
      name: "HostError",
      code: "timeout",
    });
  });
});

describe("PluginClient.on", () => {
  it("routes incoming events to subscribers and unsubscribes cleanly", () => {
    const { transport, client } = setup();
    const seen: number[] = [];
    const off = client.on<{ x: number }>("telemetry.battery", (args) => {
      seen.push(args.x);
    });
    transport.pushFromHost({
      id: "evt-1",
      type: "event",
      method: "telemetry.battery",
      capability: "telemetry.subscribe.battery",
      args: { x: 1 },
      version: PROTOCOL_VERSION,
    });
    transport.pushFromHost({
      id: "evt-2",
      type: "event",
      method: "telemetry.battery",
      capability: "telemetry.subscribe.battery",
      args: { x: 2 },
      version: PROTOCOL_VERSION,
    });
    off();
    transport.pushFromHost({
      id: "evt-3",
      type: "event",
      method: "telemetry.battery",
      capability: "telemetry.subscribe.battery",
      args: { x: 3 },
      version: PROTOCOL_VERSION,
    });
    expect(seen).toEqual([1, 2]);
  });
});

describe("PluginClient.dispose", () => {
  it("rejects every in-flight request with a disposed HostError", async () => {
    const { client } = setup();
    const promise = client.request("never", "", {}, { timeoutMs: 0 });
    client.dispose();
    await expect(promise).rejects.toBeInstanceOf(HostError);
  });
});
