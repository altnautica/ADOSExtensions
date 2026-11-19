/** A stub node agent for client tests: `fetch` is a recording mock. */

import { vi, type Mock } from "vitest";
import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

export type AgentFetch = Mock<(path: string, init?: RequestInit) => Promise<Response>>;

export interface StubAgent {
  agent: InlineAgentApi;
  fetch: AgentFetch;
}

/** An agent whose `fetch` answers each call with `reply(path, init)`. */
export function stubAgent(
  reply: (path: string, init?: RequestInit) => Promise<Response>,
): StubAgent {
  const fetch: AgentFetch = vi.fn(reply);
  const agent: InlineAgentApi = {
    fetch,
    websocket: () => Promise.reject(new Error("no socket in this test")),
  };
  return { agent, fetch };
}

/** A JSON reply with `status`. */
export function json(status: number, body: unknown): Promise<Response> {
  return Promise.resolve(
    new Response(JSON.stringify(body), {
      status,
      headers: { "Content-Type": "application/json" },
    }),
  );
}

/** The `n`th recorded call's path and init. */
export function callAt(fetch: AgentFetch, n = 0): { path: string; init: RequestInit } {
  const args = fetch.mock.calls[n];
  if (!args) throw new Error(`agent call ${n} was not made`);
  return { path: args[0], init: args[1] ?? {} };
}
