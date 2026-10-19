/**
 * @module SessionsPage
 * @description A capture device's recorded sessions (Visor and drone node
 * surface `sessions`): the session table with upload progress, then for the
 * selected session the preset picker, the reconstruction jobs on its node
 * with their pipeline, the world viewer and tour download for a finished
 * job, and change detection between two sessions of a site.
 */

import { useMemo, useState } from "react";
import { FolderOpen } from "lucide-react";

import { ViewerLoading } from "../../components/atlas/viewers/ViewerLoading";
import { ChangeDetection } from "../../components/sessions/ChangeDetection";
import { JobPipelineView } from "../../components/sessions/JobPipelineView";
import { JobWorldPanel } from "../../components/sessions/JobWorldPanel";
import { PresetPicker } from "../../components/sessions/PresetPicker";
import { SessionTable } from "../../components/sessions/SessionTable";
import { Button } from "../../components/ui/Button";
import { Section } from "../../components/ui/Section";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "../../hooks/use-compute-jobs";
import { useSessions } from "../../hooks/use-sessions";
import { CalmState } from "../workstation/CalmState";

const t = translator("sessions");

export function SessionsPage() {
  const { client, sessions, rates, loading, absent, refresh } = useSessions();
  const [selectedId, setSelectedId] = useState<string | null>(null);
  /** Jobs this page started, per session, until the node lists them. */
  const [launched, setLaunched] = useState<Record<string, { node: string | null; jobId: string | null }>>({});
  const [openJobId, setOpenJobId] = useState<string | null>(null);

  const session = sessions.find((s) => s.sessionId === selectedId) ?? sessions[0] ?? null;
  const nodeId = (session && (launched[session.sessionId]?.node ?? session.upload.node)) ?? null;
  const { jobs, client: nodeClient, unreachable } = useComputeJobs(nodeId);

  const sessionJobs = useMemo(() => {
    if (!session) return [];
    const started = launched[session.sessionId]?.jobId;
    return jobs
      .filter((j) => j.sessionId === session.sessionId || j.id === started)
      .sort((a, b) => b.createdMs - a.createdMs);
  }, [jobs, session, launched]);
  const changeJobs = jobs.filter((j) => j.kind === "change").sort((a, b) => b.createdMs - a.createdMs);
  const onNode = sessions.filter((s) => s.upload.state === "committed" && s.upload.node === nodeId);

  if (loading) return <ViewerLoading label={t("loading")} />;
  if (absent) return <CalmState icon={FolderOpen} message={t("absent")} />;
  if (sessions.length === 0 || !session) return <CalmState icon={FolderOpen} message={t("empty")} />;

  return (
    <div className="we:flex we:flex-col we:gap-3 we:p-3">
      <Section title={t("title")}>
        <SessionTable
          client={client}
          sessions={sessions}
          rates={rates}
          selectedId={session.sessionId}
          onSelect={(id) => {
            setSelectedId(id);
            setOpenJobId(null);
          }}
          onChanged={refresh}
        />
      </Section>

      <Section title={t("reconstructTitle", { id: session.sessionId })}>
        <PresetPicker
          client={client}
          sessionId={session.sessionId}
          disabled={session.upload.state !== "committed"}
          onStarted={(job) => setLaunched((m) => ({ ...m, [session.sessionId]: job }))}
        />
        {session.upload.state !== "committed" && <p className="we:text-[11px] we:text-text-tertiary">{t("uploadFirst")}</p>}
        {nodeId && unreachable && <p className="we:text-[11px] we:text-status-warning">{t("nodeUnreachable", { node: nodeId })}</p>}
        {sessionJobs.map((j) => (
          <div key={j.id} className="we:flex we:items-start we:gap-2 we:border-t we:border-border-default we:pt-2">
            <div className="we:flex-1">
              <JobPipelineView job={j} />
            </div>
            {j.state === "completed" && (
              <Button size="sm" variant={openJobId === j.id ? "primary" : "secondary"} onClick={() => setOpenJobId(openJobId === j.id ? null : j.id)}>
                {t("openViewer")}
              </Button>
            )}
          </div>
        ))}
      </Section>

      {openJobId && nodeClient && (
        <Section title={t("viewerTitle")}>
          <JobWorldPanel client={nodeClient} jobId={openJobId} />
        </Section>
      )}

      <Section title={t("changeTitle")}>
        <ChangeDetection sessions={onNode} nodeClient={nodeClient} />
        {changeJobs.map((j) => (
          <div key={j.id} className="we:flex we:items-start we:gap-2 we:border-t we:border-border-default we:pt-2">
            <div className="we:flex-1">
              <JobPipelineView job={j} />
            </div>
            {j.state === "completed" && (
              <Button size="sm" variant="secondary" onClick={() => setOpenJobId(j.id)}>
                {t("openViewer")}
              </Button>
            )}
          </div>
        ))}
      </Section>
    </div>
  );
}
