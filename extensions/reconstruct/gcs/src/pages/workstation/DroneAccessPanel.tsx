/**
 * @module DroneAccessPanel
 * @description A workstation's "Drone access" view: for every paired drone
 * and ground station, whether it holds a credential from this workstation, so
 * its lanes here (capture ingest, live stream, perception offload,
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

import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { RECONSTRUCT_PLUGIN, agentOn, useHost } from "@altnautica/mapping-ui/host/context";
import { translator } from "../../host/i18n";
import { usePairedNodes } from "@altnautica/mapping-ui/hooks/use-paired-nodes";
import { ComputeAgentClient } from "@altnautica/mapping-ui/lib/agent/compute-client";
import {
  listNodeCredentials,
  revokeNodeCredential,
  type ListedNodeCredential,
} from "@altnautica/mapping-ui/lib/agent/node-credential-client";
import { RECONSTRUCT_CREDENTIALS, sameLanes } from "@altnautica/mapping-ui/lib/nodes/credential-targets";
import { provisionAndRecord } from "@altnautica/mapping-ui/lib/nodes/workstation-provisioning";
import { cn } from "@altnautica/mapping-ui/lib/utils";
import { useWorkstationLinkStore, workstationLinkKey } from "@altnautica/mapping-ui/stores/workstation-link-store";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";

const t = translator("node.droneAccess");

export function DroneAccessPanel({ deviceId }: { deviceId: string | null }) {
  const host = useHost();
  const nodes = usePairedNodes();
  const workstation = nodes.find((n) => n.deviceId === deviceId);
  const target = RECONSTRUCT_CREDENTIALS;
  const peers = nodes.filter((n) => n.deviceId !== deviceId && target.lanesFor(n.profile) !== null);
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
    <div className="rec:flex rec:h-full rec:flex-col">
      <div className="rec:border-b rec:border-border-default rec:p-2 rec:text-[11px] rec:text-text-tertiary">
        {current && current.issued === null ? t("unreachable") : t("intro")}
      </div>
      <ul className="rec:flex-1 rec:divide-y rec:divide-border-default rec:overflow-y-auto">
        {peers.map((peer) => (
          <PeerRow
            key={peer.deviceId}
            workstation={workstation}
            client={client}
            peer={peer}
            issued={
              issued?.find(
                (c) => c.peerDeviceId === peer.deviceId && sameLanes(c.lanes, target.lanesFor(peer.profile) ?? []),
              ) ?? null
            }
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
    const target = RECONSTRUCT_CREDENTIALS;
    await provisionAndRecord(
      { workstation, peer, lanes: target.lanesFor(peer.profile) ?? [] },
      (id) => agentOn(host, id, RECONSTRUCT_PLUGIN),
      (id) => agentOn(host, id, target.plugin),
      target.installPath,
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
    <li className="rec:flex rec:items-center rec:gap-3 rec:px-3 rec:py-2">
      <span
        className={cn(
          "rec:h-2 rec:w-2 rec:shrink-0 rec:rounded-full",
          healthy ? "rec:bg-status-success" : "rec:bg-status-warning",
        )}
        aria-hidden
      />
      <div className="rec:min-w-0 rec:flex-1">
        <div className="rec:truncate rec:text-xs rec:text-text-primary">
          {peer.name}
          <span className="rec:ml-2 rec:text-[10px] rec:uppercase rec:tracking-wide rec:text-text-tertiary">
            {t(peer.profile === "drone" ? "profileDrone" : "profileGroundStation")}
          </span>
        </div>
        <div className="rec:truncate rec:text-[11px] rec:text-text-secondary">{workstationFact}</div>
        <div className="rec:truncate rec:text-[11px] rec:text-text-tertiary">{nodeFact}</div>
        {!peer.reachable && (
          <div className="rec:truncate rec:text-[11px] rec:text-status-warning">{t("peerUnreachable")}</div>
        )}
        {revokeError && (
          <div className="rec:truncate rec:text-[11px] rec:text-status-error">
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
