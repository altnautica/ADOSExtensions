/**
 * @module DroneAccessPanel
 * @description A workstation's "Drone access" view: for every paired drone
 * and ground station, whether it holds a credential from this workstation, so
 * its lanes here (Atlas ingest and world stream, perception offload,
 * artifacts) are admitted.
 *
 * Two facts per node, each read from where it is true: what the workstation
 * says it issued (read live; a credential issued before the workstation was
 * re-paired admits nothing and reads as stale), and what this GCS recorded
 * installing on the node. Provisioning runs automatically in the background;
 * the operator can re-provision a node now or revoke its credential, which
 * stays revoked until re-provisioned.
 */

import { useEffect, useMemo, useState } from "react";
import { KeyRound } from "lucide-react";
import type { InlineNodeSummary } from "@altnautica/plugin-sdk/inline";

import { Button } from "../../components/ui/Button";
import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { usePairedNodes } from "../../hooks/use-paired-nodes";
import { ComputeAgentClient } from "../../lib/agent/compute-client";
import {
  listNodeCredentials,
  revokeNodeCredential,
  type ListedNodeCredential,
} from "../../lib/agent/node-credential-client";
import { lanesForPeer, provisionAndRecord } from "../../lib/nodes/workstation-provisioning";
import { cn } from "../../lib/utils";
import { useWorkstationLinkStore, workstationLinkKey } from "../../stores/workstation-link-store";
import { CalmState } from "./CalmState";

const t = translator("atlas.droneAccess");

export function DroneAccessPanel({ deviceId }: { deviceId: string | null }) {
  const host = useHost();
  const nodes = usePairedNodes();
  const workstation = nodes.find((n) => n.deviceId === deviceId);
  const peers = nodes.filter((n) => n.deviceId !== deviceId && lanesForPeer(n.profile) !== null);
  const links = useWorkstationLinkStore((s) => s.links);
  const inFlight = useWorkstationLinkStore((s) => s.inFlight);
  const client = useMemo(
    () => (deviceId ? new ComputeAgentClient(deviceId, host.agent) : null),
    [deviceId, host.agent],
  );

  // What the workstation says it issued, tagged with the client it was read
  // through so a node switch never shows the previous node's list;
  // `issued: null` means the workstation did not answer.
  const [listing, setListing] = useState<{
    client: ComputeAgentClient;
    issued: ListedNodeCredential[] | null;
  } | null>(null);
  const [refreshTick, setRefreshTick] = useState(0);

  // Re-read whenever a provisioning settles or a row acted, so the view
  // follows the background loop and the operator.
  useEffect(() => {
    if (!client) return;
    let cancelled = false;
    void listNodeCredentials(client).then((issued) => {
      if (!cancelled) setListing({ client, issued });
    });
    return () => {
      cancelled = true;
    };
  }, [client, links, refreshTick]);

  if (!client || !workstation) return <CalmState icon={KeyRound} message={t("localOnly")} />;
  if (peers.length === 0) return <CalmState icon={KeyRound} message={t("noPeers")} />;

  const current = listing?.client === client ? listing : null;
  const issued = current?.issued ?? null;

  return (
    <div className="we:flex we:h-full we:flex-col">
      <div className="we:border-b we:border-border-default we:p-2 we:text-[11px] we:text-text-tertiary">
        {current && current.issued === null ? t("unreachable") : t("intro")}
      </div>
      <ul className="we:flex-1 we:divide-y we:divide-border-default we:overflow-y-auto">
        {peers.map((peer) => (
          <PeerRow
            key={peer.deviceId}
            workstation={workstation}
            client={client}
            peer={peer}
            issued={issued?.find((c) => c.peerDeviceId === peer.deviceId) ?? null}
            issuedKnown={issued !== null}
            busy={Boolean(inFlight[workstationLinkKey(workstation.deviceId, peer.deviceId)])}
            onChanged={() => setRefreshTick((n) => n + 1)}
          />
        ))}
      </ul>
    </div>
  );
}

function PeerRow({
  workstation,
  client,
  peer,
  issued,
  issuedKnown,
  busy,
  onChanged,
}: {
  workstation: InlineNodeSummary;
  client: ComputeAgentClient;
  peer: InlineNodeSummary;
  issued: ListedNodeCredential | null;
  issuedKnown: boolean;
  busy: boolean;
  onChanged: () => void;
}) {
  const host = useHost();
  const link = useWorkstationLinkStore((s) => s.links[workstationLinkKey(workstation.deviceId, peer.deviceId)]);
  const [revokeError, setRevokeError] = useState<string | null>(null);

  let workstationFact: string;
  if (!issuedKnown) workstationFact = t("issuedUnknown");
  else if (issued === null) workstationFact = t("notIssued");
  else if (issued.current) workstationFact = t("issuedAt", { at: new Date(issued.createdAtMs).toLocaleString() });
  else workstationFact = t("issuedStale");

  let nodeFact: string;
  if (busy) nodeFact = t("provisioning");
  else if (link === undefined) nodeFact = t("notInstalled");
  else if (link.state === "provisioned") nodeFact = t("installedAt", { at: new Date(link.at).toLocaleString() });
  else if (link.state === "revoked") nodeFact = t("revoked");
  else nodeFact = t("failed", { error: link.error ?? "" });

  const healthy = !busy && link?.state === "provisioned" && issued !== null && issued.current;

  const reprovision = async () => {
    setRevokeError(null);
    await provisionAndRecord(
      { workstation, peer, lanes: lanesForPeer(peer.profile) ?? [] },
      (id) => host.nodes.agent(id),
    );
    onChanged();
  };

  const revoke = async () => {
    if (!issued) return;
    setRevokeError(null);
    const r = await revokeNodeCredential(client, issued.id);
    if (!r.ok) {
      setRevokeError(r.message);
      return;
    }
    useWorkstationLinkStore.getState().record({
      workstationDeviceId: workstation.deviceId,
      peerDeviceId: peer.deviceId,
      state: "revoked",
      credentialId: issued.id,
      at: Date.now(),
    });
    onChanged();
  };

  return (
    <li className="we:flex we:items-center we:gap-3 we:px-3 we:py-2">
      <span
        className={cn(
          "we:h-2 we:w-2 we:shrink-0 we:rounded-full",
          healthy ? "we:bg-status-success" : "we:bg-status-warning",
        )}
        aria-hidden
      />
      <div className="we:min-w-0 we:flex-1">
        <div className="we:truncate we:text-xs we:text-text-primary">
          {peer.name}
          <span className="we:ml-2 we:text-[10px] we:uppercase we:tracking-wide we:text-text-tertiary">
            {t(peer.profile === "drone" ? "profileDrone" : "profileGroundStation")}
          </span>
        </div>
        <div className="we:truncate we:text-[11px] we:text-text-secondary">{workstationFact}</div>
        <div className="we:truncate we:text-[11px] we:text-text-tertiary">{nodeFact}</div>
        {!peer.reachable && (
          <div className="we:truncate we:text-[11px] we:text-status-warning">{t("peerUnreachable")}</div>
        )}
        {revokeError && (
          <div className="we:truncate we:text-[11px] we:text-status-error">
            {t("revokeFailed", { error: revokeError })}
          </div>
        )}
      </div>
      <Button
        size="sm"
        variant="secondary"
        disabled={busy || !peer.reachable}
        onClick={() => void reprovision()}
      >
        {link === undefined ? t("provision") : t("reprovision")}
      </Button>
      <Button size="sm" variant="ghost" disabled={busy || issued === null} onClick={() => void revoke()}>
        {t("revoke")}
      </Button>
    </li>
  );
}
