/**
 * @module calibration/RigEntryCard
 * @description A measured / CAD-entry wizard step: the LiDAR mounting
 * extrinsic (translation + roll/pitch/yaw in body FLU) or the GNSS antenna
 * lever arm, entered by the operator and pushed to the device as a new
 * calibration version of the active rig. No solver runs.
 */

import { useState } from "react";
import { Ruler } from "lucide-react";

import { translator } from "../../host/i18n";
import type { CaptureClient } from "../../lib/agent/capture-client";
import { isPlanarLidar, type EntryTarget } from "../../lib/capture/calibration";
import {
  manualCalibVersion,
  transformFromXyzRpy,
  withExtrinsic,
  withLeverArm,
  xyzRpyFromTransform,
  type Xyz,
} from "../../lib/capture/rig-entry";
import type { Rig, RigSensor } from "../../lib/contract/generated";
import { Button } from "../ui/Button";
import { useToast } from "../ui/Toast";

const t = translator("calibration");

function TripleInput({ label, value, onChange, unit }: { label: string; value: Xyz; onChange: (v: Xyz) => void; unit: string }) {
  return (
    <fieldset className="we:flex we:items-center we:gap-1 we:text-[11px] we:text-text-tertiary">
      <legend className="we:sr-only">{label}</legend>
      <span className="we:w-20">{label}</span>
      {value.map((v, i) => (
        <input
          key={i}
          type="number"
          step="0.001"
          aria-label={`${label} ${"xyz"[i]}`}
          value={Number.isFinite(v) ? v : 0}
          onChange={(e) => {
            const next = [...value] as Xyz;
            next[i] = Number(e.target.value);
            onChange(next);
          }}
          className="we:h-7 we:w-20 we:rounded we:border we:border-border-default we:bg-bg-primary we:px-1 we:font-mono we:text-xs we:text-text-primary"
        />
      ))}
      <span>{unit}</span>
    </fieldset>
  );
}

/** The editable values of one sensor. Mounted per sensor (keyed by its id) so
 * picking another sensor starts from that sensor's stored values. */
function SensorFields({
  target,
  rig,
  sensor,
  device,
  onApplied,
}: {
  target: EntryTarget;
  rig: Rig;
  sensor: RigSensor;
  device: CaptureClient;
  onApplied: () => void;
}) {
  const { toast } = useToast();
  const current = xyzRpyFromTransform(sensor.T_body_sensor);
  const [xyz, setXyz] = useState<Xyz>(
    target === "gnss_lever_arm" ? ((sensor.lever_arm_m as Xyz | undefined) ?? current.xyz) : current.xyz,
  );
  const [rpy, setRpy] = useState<Xyz>(current.rpy);
  const [busy, setBusy] = useState(false);

  const save = async () => {
    const version = manualCalibVersion(new Date());
    const next =
      target === "lidar_imu"
        ? withExtrinsic(rig, sensor.id, transformFromXyzRpy(xyz, rpy), version)
        : withLeverArm(rig, sensor.id, xyz, version);
    setBusy(true);
    const res = await device.putRig(next);
    setBusy(false);
    if (res.ok) {
      toast(t("applied", { version }), "success");
      onApplied();
    } else toast(res.message, "error");
  };

  return (
    <>
      <TripleInput label={target === "lidar_imu" ? t("translation") : t("leverArm")} value={xyz} onChange={setXyz} unit="m" />
      {target === "lidar_imu" && <TripleInput label={t("rotation")} value={rpy} onChange={setRpy} unit="° (r, p, y)" />}
      <Button size="sm" className="we:self-start" loading={busy} onClick={() => void save()}>
        {t("saveToRig")}
      </Button>
    </>
  );
}

export function RigEntryCard({
  index,
  target,
  rig,
  device,
  onApplied,
}: {
  index: number;
  target: EntryTarget;
  rig: Rig;
  device: CaptureClient;
  onApplied: () => void;
}) {
  // A planar LiDAR's mount comes from the LiDAR–camera board job, not this entry.
  const sensors = rig.sensors.filter((s) =>
    target === "lidar_imu" ? s.kind === "lidar" && !isPlanarLidar(s) : s.kind === "gnss",
  );
  const [sensorId, setSensorId] = useState(sensors[0]?.id ?? "");
  const sensor = sensors.find((s) => s.id === sensorId) ?? sensors[0];

  return (
    <li className="we:flex we:flex-col we:gap-2 we:border-t we:border-border-default we:pt-2" data-step={target}>
      <div className="we:flex we:items-center we:gap-2 we:text-xs">
        <Ruler size={12} className="we:text-text-tertiary" />
        <span className="we:font-medium we:text-text-primary">
          {index + 1}. {t(`step.${target}`)}
        </span>
        <span className="we:text-[11px] we:text-text-tertiary">{t("measured")}</span>
      </div>
      <p className="we:text-[11px] we:text-text-tertiary">{t(`hint.${target}`)}</p>
      {sensors.length > 1 && (
        <select
          aria-label={t("sensor")}
          value={sensor?.id ?? ""}
          onChange={(e) => setSensorId(e.target.value)}
          className="we:h-7 we:w-40 we:rounded we:border we:border-border-default we:bg-bg-primary we:px-1 we:text-xs"
        >
          {sensors.map((s) => (
            <option key={s.id} value={s.id}>
              {s.id}
            </option>
          ))}
        </select>
      )}
      {sensor && (
        <SensorFields key={sensor.id} target={target} rig={rig} sensor={sensor} device={device} onApplied={onApplied} />
      )}
    </li>
  );
}
