/**
 * Which paired nodes get a workstation credential, and when again. A drone
 * paired alongside a workstation must be provisioned without anyone asking; a
 * workstation that no longer lists the installed credential as current (it was
 * re-paired or wiped) must provision again; a failure retries on a fixed
 * interval; and an operator's revoke is never undone behind their back. The
 * end-to-end act issues on the workstation, installs the issued token on the
 * drone, and records both.
 */

import { afterEach, describe, expect, it, vi } from "vitest";
import type { InlineAgentApi, InlineNodeSummary } from "@altnautica/plugin-sdk/inline";

// The link store is persisted; bind a deterministic in-memory localStorage
// before the store module is imported (the runtime's own may be absent).
vi.hoisted(() => {
  const mem = new Map<string, string>();
  Object.defineProperty(globalThis, "localStorage", {
    configurable: true,
    writable: true,
    value: {
      getItem: (k: string) => mem.get(k) ?? null,
      setItem: (k: string, v: string) => void mem.set(k, v),
      removeItem: (k: string) => void mem.delete(k),
      clear: () => mem.clear(),
      key: (i: number) => Array.from(mem.keys())[i] ?? null,
      get length() {
        return mem.size;
      },
    },
  });
});

import {
  isComputeProfile,
  lanesForPeer,
  planProvisioning,
  provisionAndRecord,
  PROVISION_RETRY_MS,
  type ProvisionPair,
} from "../../../src/lib/nodes/workstation-provisioning";
import type { ListedNodeCredential } from "../../../src/lib/agent/node-credential-client";
import {
  useWorkstationLinkStore,
  workstationLinkKey,
  type WorkstationLink,
} from "../../../src/stores/workstation-link-store";
import { callAt, json, stubAgent, type StubAgent } from "../../helpers/agent";

function node(
  deviceId: string,
  profile: InlineNodeSummary["profile"],
  reachable = true,
): InlineNodeSummary {
  return { deviceId, name: deviceId, profile, reachable, lanHost: `${deviceId}.local` };
}

const WS = node("ws", "workstation");
const DRONE = node("drone", "drone");
const GS = node("gs", "ground-station");

function link(over: Partial<WorkstationLink>): WorkstationLink {
  return {
    workstationDeviceId: "ws",
    peerDeviceId: "drone",
    state: "provisioned",
    credentialId: "CRED1",
    at: 1000,
    ...over,
  };
}

function listed(id: string, current: boolean): ListedNodeCredential {
  return { id, peerDeviceId: "drone", lanes: ["atlas.ingest"], createdAtMs: 1, current };
}

const KEY = workstationLinkKey("ws", "drone");

function pairs(plan: ProvisionPair[]): string[][] {
  return plan.map((p) => [p.workstation.deviceId, p.peer.deviceId]);
}

describe("profiles", () => {
  it("treats workstation and compute as issuers, and gives them no lanes", () => {
    expect(isComputeProfile("workstation")).toBe(true);
    expect(isComputeProfile("compute")).toBe(true);
    expect(isComputeProfile("drone")).toBe(false);
    expect(lanesForPeer("workstation")).toBeNull();
    expect(lanesForPeer("compute")).toBeNull();
  });

  it("gives a ground station the ingest lane alone and a drone every lane", () => {
    expect(lanesForPeer("ground-station")).toEqual(["atlas.ingest"]);
    expect(lanesForPeer("drone")).toEqual(
      expect.arrayContaining(["atlas.ingest", "atlas.world", "offload.stream", "jobs.submit"]),
    );
  });
});

describe("planProvisioning", () => {
  it("provisions every drone and ground station against every workstation", () => {
    const plan = planProvisioning([WS, DRONE, GS, node("cn", "compute")], {}, {}, 0);
    expect(pairs(plan)).toEqual([
      ["ws", "drone"],
      ["ws", "gs"],
      ["cn", "drone"],
      ["cn", "gs"],
    ]);
    // A ground station only relays Atlas events: it gets the ingest lane alone.
    expect(plan[0]?.lanes).toContain("jobs.submit");
    expect(plan[1]?.lanes).toEqual(["atlas.ingest"]);
  });

  it("needs a workstation and a peer, both reachable", () => {
    expect(planProvisioning([DRONE, GS], {}, {}, 0)).toEqual([]);
    expect(planProvisioning([WS], {}, {}, 0)).toEqual([]);
    expect(planProvisioning([node("ws", "workstation", false), DRONE], {}, {}, 0)).toEqual([]);
    expect(planProvisioning([WS, node("drone", "drone", false)], {}, {}, 0)).toEqual([]);
  });

  it("leaves a provisioned link alone when no listing was read", () => {
    const links = { [KEY]: link({}) };
    expect(planProvisioning([WS, DRONE], links, {}, 99_999)).toEqual([]);
    expect(planProvisioning([WS, DRONE], links, {}, 99_999, { ws: null })).toEqual([]);
  });

  it("leaves a provisioned link alone while the workstation lists its credential as current", () => {
    const links = { [KEY]: link({}) };
    expect(
      planProvisioning([WS, DRONE], links, {}, 0, { ws: [listed("CRED1", true)] }),
    ).toEqual([]);
  });

  it("re-provisions when the workstation no longer lists the credential as current", () => {
    const links = { [KEY]: link({}) };
    // Re-paired workstation: the credential is still listed, but not current.
    expect(
      pairs(planProvisioning([WS, DRONE], links, {}, 0, { ws: [listed("CRED1", false)] })),
    ).toEqual([["ws", "drone"]]);
    // Wiped workstation: the credential is gone.
    expect(pairs(planProvisioning([WS, DRONE], links, {}, 0, { ws: [] }))).toEqual([
      ["ws", "drone"],
    ]);
  });

  it("retries a failed link on the fixed interval", () => {
    const links = { [KEY]: link({ state: "failed", at: 1000 }) };
    expect(planProvisioning([WS, DRONE], links, {}, 1000 + PROVISION_RETRY_MS - 1)).toEqual([]);
    expect(planProvisioning([WS, DRONE], links, {}, 1000 + PROVISION_RETRY_MS)).toHaveLength(1);
  });

  it("never re-provisions a revoked link on its own, even when the workstation lost it", () => {
    const links = { [KEY]: link({ state: "revoked" }) };
    expect(planProvisioning([WS, DRONE], links, {}, 99_999)).toEqual([]);
    expect(planProvisioning([WS, DRONE], links, {}, 99_999, { ws: [] })).toEqual([]);
  });

  it("skips a link already being provisioned", () => {
    expect(planProvisioning([WS, DRONE], {}, { [KEY]: true }, 0)).toEqual([]);
  });
});

describe("provisionAndRecord", () => {
  afterEach(() => {
    useWorkstationLinkStore.setState({ links: {}, inFlight: {} });
  });

  const ISSUED = {
    id: "CRED1",
    peer_device_id: "drone",
    lanes: ["atlas.ingest", "jobs.submit"],
    created_at_ms: 5,
    credential: "nc1.CRED1.SECRET",
    workstation_node_id: "compute-abc",
  };

  function agents(ws: StubAgent, peer: StubAgent): (deviceId: string) => InlineAgentApi {
    return (deviceId) => (deviceId === "ws" ? ws.agent : peer.agent);
  }

  it("issues on the workstation, installs the same token on the drone, and records it", async () => {
    const ws = stubAgent(() => json(201, ISSUED));
    const drone = stubAgent(() => json(200, { installed: true }));
    const result = await provisionAndRecord(
      { workstation: WS, peer: DRONE, lanes: ["atlas.ingest", "jobs.submit"] },
      agents(ws, drone),
    );
    expect(result.state).toBe("provisioned");
    expect(result.credentialId).toBe("CRED1");

    // Issued on the workstation's job API.
    const issue = callAt(ws.fetch);
    expect(issue.path).toBe("api/compute/node-credentials");
    expect(issue.init.method).toBe("POST");
    expect(JSON.parse(String(issue.init.body))).toEqual({
      peer_device_id: "drone",
      lanes: ["atlas.ingest", "jobs.submit"],
    });
    // Installed on the drone, filed under the id the workstation advertises.
    const install = callAt(drone.fetch);
    expect(install.path).toBe("node-credential");
    expect(JSON.parse(String(install.init.body))).toEqual({
      workstation_node_id: "compute-abc",
      credential: "nc1.CRED1.SECRET",
      lanes: ["atlas.ingest", "jobs.submit"],
    });
    expect(useWorkstationLinkStore.getState().links[KEY]?.state).toBe("provisioned");
    expect(useWorkstationLinkStore.getState().inFlight[KEY]).toBeUndefined();
  });

  it("records a workstation refusal with the side that refused, and installs nothing", async () => {
    const ws = stubAgent(() => json(409, { error: "this node is not paired" }));
    const drone = stubAgent(() => json(200, { installed: true }));
    const result = await provisionAndRecord(
      { workstation: WS, peer: DRONE, lanes: ["atlas.ingest"] },
      agents(ws, drone),
    );
    expect(result.state).toBe("failed");
    expect(result.error).toBe("workstation: this node is not paired");
    expect(drone.fetch).not.toHaveBeenCalled();
    expect(useWorkstationLinkStore.getState().links[KEY]?.state).toBe("failed");
  });

  it("names the peer when the install fails, keeping the issued credential id", async () => {
    const ws = stubAgent(() => json(201, ISSUED));
    const drone = stubAgent(() => json(500, { detail: "store is read-only" }));
    const result = await provisionAndRecord(
      { workstation: WS, peer: DRONE, lanes: ["atlas.ingest"] },
      agents(ws, drone),
    );
    expect(result).toMatchObject({
      state: "failed",
      credentialId: "CRED1",
      error: "drone: store is read-only",
    });
  });

  it("records a host refusal as a failed attempt and clears the in-flight mark", async () => {
    const result = await provisionAndRecord(
      { workstation: WS, peer: DRONE, lanes: ["atlas.ingest"] },
      () => {
        throw new Error("node_unreachable");
      },
    );
    expect(result).toMatchObject({ state: "failed", error: "node_unreachable" });
    expect(useWorkstationLinkStore.getState().inFlight[KEY]).toBeUndefined();
  });

  it("leaves a pair already in flight alone", async () => {
    useWorkstationLinkStore.getState().setInFlight(KEY, true);
    const ws = stubAgent(() => json(201, ISSUED));
    const drone = stubAgent(() => json(200, { installed: true }));
    await provisionAndRecord({ workstation: WS, peer: DRONE, lanes: ["atlas.ingest"] }, agents(ws, drone));
    expect(ws.fetch).not.toHaveBeenCalled();
    expect(useWorkstationLinkStore.getState().inFlight[KEY]).toBe(true);
  });
});
