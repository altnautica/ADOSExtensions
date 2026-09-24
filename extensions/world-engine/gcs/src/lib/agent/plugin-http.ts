/**
 * JSON exchanges with this extension's HTTP server on a node, through the
 * host's agent passthrough. Every call is bounded by a deadline and resolves
 * to the status plus the parsed body, or null on a transport failure, so a
 * poll loop degrades instead of throwing.
 */

import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

/** Default deadline for one agent request. */
export const AGENT_FETCH_TIMEOUT_MS = 6000;

/** How long the agent may take to restart a service after a config write. */
export const AGENT_SERVICE_RESTART_TIMEOUT_MS = 40_000;

/** Client deadline for a write that restarts a service: the agent's own
 * restart bound plus headroom for the reply. */
export const AGENT_SERVICE_RESTART_CLIENT_TIMEOUT_MS = AGENT_SERVICE_RESTART_TIMEOUT_MS + 5_000;

export type HttpMethod = "GET" | "POST" | "PUT" | "DELETE";

/** One reply: the HTTP status and the parsed JSON body (null when not JSON). */
export interface JsonReply {
  status: number;
  json: unknown;
}

/**
 * Send one request to `path` on the node's extension server. A body is sent as
 * JSON. Returns null when the request did not complete (unreachable node, the
 * deadline, a host refusal).
 */
export async function requestJson(
  agent: InlineAgentApi,
  path: string,
  method: HttpMethod,
  body?: unknown,
  timeoutMs: number = AGENT_FETCH_TIMEOUT_MS,
): Promise<JsonReply | null> {
  const hasBody = body !== undefined && body !== null;
  let res: Response;
  try {
    res = await agent.fetch(path, {
      method,
      headers: {
        Accept: "application/json",
        ...(hasBody ? { "Content-Type": "application/json" } : {}),
      },
      body: hasBody ? JSON.stringify(body) : undefined,
      signal: AbortSignal.timeout(timeoutMs),
    });
  } catch {
    return null;
  }
  // A misbehaving fetch can resolve undefined; treat it as a transport failure.
  if (!res) return null;
  let json: unknown = null;
  try {
    json = await res.json();
  } catch {
    json = null;
  }
  return { status: res.status, json };
}

/** The parsed body of a 2xx reply, or null on any other outcome. */
export async function requestOkJson(
  agent: InlineAgentApi,
  path: string,
  method: HttpMethod,
  body?: unknown,
): Promise<unknown | null> {
  const reply = await requestJson(agent, path, method, body);
  return reply && reply.status >= 200 && reply.status < 300 ? reply.json : null;
}
