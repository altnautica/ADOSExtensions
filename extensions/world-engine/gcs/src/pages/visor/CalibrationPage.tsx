/**
 * @module CalibrationPage
 * @description The capture rig's calibration (Visor node surface
 * `calibration`): the wizard — IMU noise → camera intrinsics → camera–IMU
 * (node `calibrate` jobs on recorded sessions) → LiDAR–IMU (measured entry) →
 * LiDAR–camera (node job) → GNSS lever arm (measured / CAD entry), each step
 * shown only when the rig carries that sensor — and the device's stored
 * calibration versions with the active one and an activate action that pushes
 * that version's rig to the device.
 */

import { useCallback, useEffect, useMemo, useState } from "react";
import { Crosshair } from "lucide-react";

import { CalibStepCard, type StepProgress } from "../../components/calibration/CalibStepCard";
import { RigEntryCard } from "../../components/calibration/RigEntryCard";
import { ViewerLoading } from "../../components/atlas/viewers/ViewerLoading";
import { Button } from "../../components/ui/Button";
import { Section } from "../../components/ui/Section";
import { useToast } from "../../components/ui/Toast";
import { translator } from "../../host/i18n";
import { usePairedNodes } from "../../hooks/use-paired-nodes";
import { useSessions } from "../../hooks/use-sessions";
import { wizardFor, type CalibStore, type CalibTarget } from "../../lib/capture/calibration";
import type { Rig } from "../../lib/contract/generated";
import { CalmState } from "../workstation/CalmState";

const t = translator("calibration");

const IDLE: StepProgress = { sessionId: null, recording: false, jobId: null };

export function CalibrationPage() {
  const { client, sessions, refresh: refreshSessions } = useSessions();
  const { toast } = useToast();
  const nodes = usePairedNodes();
  const [store, setStore] = useState<CalibStore | null | undefined>(undefined);
  const [rig, setRig] = useState<Rig | null>(null);
  const [progress, setProgress] = useState<Partial<Record<CalibTarget, StepProgress>>>({});

  const load = useCallback(async () => {
    const s = await client.getCalib();
    setStore(s);
    setRig(s?.active ? await client.getCalibRig(s.active) : null);
  }, [client]);
  useEffect(() => {
    void load();
  }, [load]);

  /** The node a calibration session goes to: the one it uploaded to, else the
   * first paired workstation or compute node. */
  const fallbackNode = useMemo(
    () => nodes.find((n) => n.profile === "workstation" || n.profile === "compute")?.deviceId ?? null,
    [nodes],
  );
  const wizard = useMemo(
    () =>
      wizardFor({
        hasLidar: rig?.sensors.some((x) => x.kind === "lidar") ?? false,
        hasGnss: rig?.sensors.some((x) => x.kind === "gnss") ?? false,
      }),
    [rig],
  );

  if (store === undefined) return <ViewerLoading label={t("loading")} />;
  if (store === null) return <CalmState icon={Crosshair} message={t("absent")} />;

  const activate = async (version: string) => {
    const versionRig = await client.getCalibRig(version);
    if (!versionRig) {
      toast(t("versionUnreadable"), "error");
      return;
    }
    const res = await client.putRig(versionRig);
    if (res.ok) toast(t("applied", { version }), "success");
    else toast(res.message, "error");
    void load();
  };

  return (
    <div className="we:flex we:flex-col we:gap-3 we:p-3">
      <Section title={t("wizard")}>
        <p className="we:text-[11px] we:text-text-tertiary">
          {t("rig", { rig: store.rigId ?? "–", version: store.active ?? "–" })}
        </p>
        <ol className="we:flex we:flex-col we:gap-2">
          {wizard.map((w, i) => {
            if (w.kind === "entry") {
              return rig ? (
                <RigEntryCard key={w.target} index={i} target={w.target} rig={rig} device={client} onApplied={() => void load()} />
              ) : null;
            }
            const step = w.step;
            const p = progress[step.target] ?? IDLE;
            const session = sessions.find((s) => s.sessionId === p.sessionId) ?? null;
            return (
              <CalibStepCard
                key={step.target}
                index={i}
                step={step}
                progress={p}
                onProgress={(next) => {
                  setProgress((m) => ({ ...m, [step.target]: next }));
                  refreshSessions();
                }}
                device={client}
                session={session}
                nodeId={session?.upload.node ?? fallbackNode}
                onApplied={() => void load()}
              />
            );
          })}
        </ol>
        {!rig && <p className="we:text-[11px] we:text-text-tertiary">{t("noActiveRig")}</p>}
      </Section>
      <Section title={t("versions")}>
        {store.versions.length === 0 ? (
          <p className="we:text-[11px] we:text-text-tertiary">{t("noVersions")}</p>
        ) : (
          <ul className="we:flex we:flex-col we:gap-1 we:text-[11px]">
            {store.versions.map((v) => (
              <li key={v.calibVersion} className="we:flex we:items-center we:gap-2" data-version={v.calibVersion}>
                <span className="we:font-mono we:text-text-secondary">{v.calibVersion}</span>
                {v.calibVersion === store.active && <span className="we:text-status-success">{t("active")}</span>}
                <span className="we:text-text-tertiary">{v.createdUtc ?? ""}</span>
                {v.reprojectionRmsPx !== null && <span className="we:font-mono">{v.reprojectionRmsPx.toFixed(2)} px</span>}
                {v.calibVersion !== store.active && (
                  <Button size="sm" variant="ghost" className="we:ml-auto" onClick={() => void activate(v.calibVersion)}>
                    {t("activate")}
                  </Button>
                )}
              </li>
            ))}
          </ul>
        )}
      </Section>
    </div>
  );
}
