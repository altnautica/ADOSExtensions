import { describe, it, expect, vi } from "vitest";

import { createPluginContext } from "../src/api";
import { PluginClient } from "../src/client";
import { HostError, PROTOCOL_VERSION, type RpcEnvelope } from "../src/protocol";
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

describe("operator-confirmed calls", () => {
  it("wait past the default deadline for the host's answer", async () => {
    vi.useFakeTimers();
    try {
      const { transport, client } = setup();
      const ctx = createPluginContext({ client });
      transport.onPluginSend = (env) => {
        // The operator reads the prompt for 8 s, then approves.
        setTimeout(() => {
          transport.pushFromHost({
            id: env.id,
            type: "response",
            method: env.method,
            capability: env.capability,
            args: { ok: true },
            version: PROTOCOL_VERSION,
          });
        }, 8_000);
      };
      const sent = ctx.command.send("land");
      const written = ctx.mission.write({ missionId: "m1" });
      await vi.advanceTimersByTimeAsync(8_000);
      await expect(sent).resolves.toEqual({ ok: true });
      await expect(written).resolves.toEqual({ ok: true });
    } finally {
      vi.useRealTimers();
    }
  });
});

describe("host refusals", () => {
  function answering(
    transport: MemoryTransport,
    result: (env: RpcEnvelope) => unknown,
  ): void {
    transport.onPluginSend = (env) => {
      queueMicrotask(() =>
        transport.pushFromHost({
          id: env.id,
          type: "response",
          method: env.method,
          capability: env.capability,
          args: result(env),
          version: PROTOCOL_VERSION,
        }),
      );
    };
  }

  it("reject every operator action the host answers with ok:false", async () => {
    const { transport, client } = setup();
    const ctx = createPluginContext({ client });
    answering(transport, () => ({ ok: false, error: "operator denied" }));
    const calls: Array<Promise<unknown>> = [
      ctx.command.send("land"),
      ctx.mission.write({ missionId: "m1" }),
      ctx.recording.mark({ label: "x" }),
      ctx.notifications.publish({
        channelId: "c",
        severity: "info",
        title: "t",
      }),
    ];
    for (const call of calls) {
      await expect(call).rejects.toMatchObject({
        name: "HostError",
        code: "refused",
        message: "operator denied",
      });
    }
  });

  it("resolve an accepted action with the host's result", async () => {
    const { transport, client } = setup();
    const ctx = createPluginContext({ client });
    answering(transport, () => ({ ok: true, result: { set: true } }));
    await expect(ctx.command.send("plugin.config.write")).resolves.toEqual({
      ok: true,
      result: { set: true },
    });
  });

  it("unwrap a records result and reject a records refusal with its reason", async () => {
    const { transport, client } = setup();
    const ctx = createPluginContext({ client });
    const sent: RpcEnvelope[] = [];
    const record = {
      collection: "jobs",
      key: "j1",
      deviceId: null,
      data: { n: 1 },
      updatedAt: 1,
      writtenBy: "gcs",
    };
    answering(transport, (env) => {
      sent.push(env);
      if (env.method === "records.put") return { ok: false, error: "limit_reached" };
      return { ok: true, result: env.method === "records.list" ? [record] : null };
    });

    await expect(ctx.records.list({ collection: "jobs", limit: 5 })).resolves.toEqual([record]);
    await expect(ctx.records.get("jobs", "missing")).resolves.toBeNull();
    await expect(ctx.records.remove("jobs", "j1")).resolves.toBeUndefined();
    await expect(ctx.records.put("jobs", "j2", { n: 2 })).rejects.toMatchObject({
      code: "refused",
      message: "limit_reached",
    });
    expect(sent.map((env) => [env.method, env.capability])).toEqual([
      ["records.list", "cloud.records"],
      ["records.get", "cloud.records"],
      ["records.remove", "cloud.records"],
      ["records.put", "cloud.records"],
    ]);
  });
});

describe("telemetry subscriptions", () => {
  it("release the host stream when the last handler unsubscribes", async () => {
    const { transport, client } = setup();
    const sent: RpcEnvelope[] = [];
    transport.onPluginSend = (env) => {
      sent.push(env);
      queueMicrotask(() =>
        transport.pushFromHost({
          id: env.id,
          type: "response",
          method: env.method,
          capability: env.capability,
          args: { ok: true },
          version: PROTOCOL_VERSION,
        }),
      );
    };
    const offA = await client.subscribeTelemetry("battery", () => {});
    const offB = await client.subscribeTelemetry("battery", () => {});
    offA();
    expect(sent.map((e) => e.method)).not.toContain("telemetry.unsubscribe");
    offB();
    offB();
    const releases = sent.filter((e) => e.method === "telemetry.unsubscribe");
    expect(releases).toHaveLength(1);
    expect(releases[0].args).toEqual({ topic: "battery" });
  });

  it("leave no handler behind when the host refuses the subscription", async () => {
    const { transport, client } = setup();
    transport.onPluginSend = (env) => {
      queueMicrotask(() =>
        transport.pushFromHost({
          id: env.id,
          type: "response",
          method: env.method,
          capability: env.capability,
          args: undefined,
          error: { code: "permission_denied", message: "no grant" },
          version: PROTOCOL_VERSION,
        }),
      );
    };
    const seen: unknown[] = [];
    await expect(
      client.subscribeTelemetry("battery", (s) => seen.push(s)),
    ).rejects.toMatchObject({ code: "permission_denied" });
    transport.pushFromHost({
      id: "e1",
      type: "event",
      method: "telemetry.battery",
      capability: "telemetry.subscribe.battery",
      args: { x: 1 },
      version: PROTOCOL_VERSION,
    });
    expect(seen).toEqual([]);
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

describe("PluginClient capability token", () => {
  it("stamps the latest host-published token on every request", async () => {
    const { transport, client } = setup();
    const sent: Array<{ token?: string }> = [];
    transport.onPluginSend = (env) => {
      sent.push(env);
      transport.pushFromHost({
        id: env.id,
        type: "response",
        method: env.method,
        capability: env.capability,
        args: null,
        version: PROTOCOL_VERSION,
      });
    };
    await client.request("ping", "", {});
    transport.pushFromHost({
      id: "t1",
      type: "event",
      method: "capability.token",
      capability: "",
      args: { token: "tok-1" },
      version: PROTOCOL_VERSION,
    });
    await client.request("ping", "", {});
    transport.pushFromHost({
      id: "t2",
      type: "event",
      method: "capability.token",
      capability: "",
      args: { token: "tok-2" },
      version: PROTOCOL_VERSION,
    });
    await client.request("ping", "", {});
    expect(sent.map((e) => e.token)).toEqual([undefined, "tok-1", "tok-2"]);
  });
});
