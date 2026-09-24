/**
 * @module pages/offload/WorkstationServing
 * @description Workstation half of the perception offload page: whether this
 * node serves offloaded perception, which detector it runs (free text, empty
 * = the engine's default), and its live GPU facts (read-only, real values
 * only) from the node's compute telemetry.
 */

import { Cpu } from "lucide-react";

import { ConfigSelectField, ConfigTextField, ReadRow } from "../../components/config/ConfigFields";
import { useHost } from "../../host/context";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../stores/compute-store";
import type { OffloadConfigProps } from "./DroneOffloadClient";
import { ENABLE_OPTIONS, perceptionT as t } from "./enable-options";

export function WorkstationServing({ config, readOnly, setValue }: OffloadConfigProps) {
  const deviceId = useHost().node.deviceId;
  const gpu = useComputeStore((s) =>
    deviceId ? (s.nodes[deviceId] ?? EMPTY_COMPUTE_NODE).gpu : null,
  );

  const hasGpu =
    gpu != null &&
    (gpu.name != null ||
      gpu.cores != null ||
      gpu.unifiedMemoryMb != null ||
      gpu.utilizationPct != null);

  return (
    <div className="we:space-y-4">
      <ConfigSelectField
        configKey="serving.enabled"
        label={t("serving.enabledLabel")}
        hint={t("serving.enabledHint")}
        options={ENABLE_OPTIONS}
        placeholder={t("enabledAutoDefault")}
        config={config}
        readOnly={readOnly}
        setValue={setValue}
      />

      <ConfigTextField
        configKey="serving.detector_model"
        label={t("serving.modelLabel")}
        hint={t("serving.modelHint")}
        placeholder={t("serving.modelDefault")}
        config={config}
        readOnly={readOnly}
        setValue={setValue}
      />

      {hasGpu ? (
        <div className="we:space-y-2 we:border-t we:border-border-default we:pt-3">
          <div className="we:flex we:items-center we:gap-1.5">
            <Cpu size={12} className="we:text-text-tertiary" aria-hidden="true" />
            <span className="we:text-xs we:text-text-secondary">{t("serving.gpuTitle")}</span>
          </div>
          {gpu.name != null ? <ReadRow label={t("serving.gpuName")} value={gpu.name} /> : null}
          {gpu.cores != null ? (
            <ReadRow label={t("serving.gpuCores")} value={String(gpu.cores)} />
          ) : null}
          {gpu.unifiedMemoryMb != null ? (
            <ReadRow
              label={t("serving.gpuMemory")}
              value={`${Math.round(gpu.unifiedMemoryMb / 1024)} GB`}
            />
          ) : null}
          {gpu.utilizationPct != null ? (
            <ReadRow label={t("serving.gpuUtil")} value={`${gpu.utilizationPct.toFixed(0)}%`} />
          ) : null}
        </div>
      ) : null}
    </div>
  );
}
