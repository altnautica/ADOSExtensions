/**
 * @module SessionsView
 * @description A capture device's recorded sessions: the session table with
 * upload progress, then for the selected session the preset picker, the
 * reconstruction jobs on its node with their pipeline, and the viewer and
 * tour download for a finished job. Comparing two sessions is Reconstruct's.
 */

import { useMemo, useState } from "react";
import { FolderOpen } from "lucide-react";

import { ViewerLoading } from "@altnautica/mapping-ui/components/model/viewers/ViewerLoading";
import { JobPipelineView } from "@altnautica/mapping-ui/components/sessions/JobPipelineView";
import { JobWorldPanel } from "@altnautica/mapping-ui/components/sessions/JobWorldPanel";
import { PresetPicker } from "../../components/sessions/PresetPicker";
import { SessionTable } from "../../components/sessions/SessionTable";
import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { Section } from "@altnautica/mapping-ui/components/ui/Section";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";
import { useSessions } from "../../hooks/use-sessions";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";

const t = translator("sessions");

export function SessionsView() {
  const { client, sessions, rates, loading, absent, stale, refresh } = useSessions();
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

  if (loading) return <ViewerLoading label={t("loading")} />;
  if (absent) return <CalmState icon={FolderOpen} message={t("absent")} />;
  if (sessions.length === 0 || !session) return <CalmState icon={FolderOpen} message={t("empty")} />;

  return (
    <div className="cap:flex cap:flex-col cap:gap-3 cap:p-3">
      {stale && (
        <p role="status" className="cap:rounded cap:bg-status-warning/15 cap:px-2 cap:py-1 cap:text-[11px] cap:text-status-warning">
          {t("stale")}
        </p>
      )}
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
        {session.upload.state !== "committed" && <p className="cap:text-[11px] cap:text-text-tertiary">{t("uploadFirst")}</p>}
        {nodeId && unreachable && <p className="cap:text-[11px] cap:text-status-warning">{t("nodeUnreachable", { node: nodeId })}</p>}
        {sessionJobs.map((j) => (
          <div key={j.id} className="cap:flex cap:items-start cap:gap-2 cap:border-t cap:border-border-default cap:pt-2">
            <div className="cap:flex-1">
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
          <JobWorldPanel key={openJobId} client={nodeClient} jobId={openJobId} />
        </Section>
      )}
    </div>
  );
}
