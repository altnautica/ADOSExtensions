/**
 * @module pages/offload/DroneOffloadClient
 * @description Drone half of the perception offload page: where this node
 * offloads perception, and which workstation it pins (empty = auto-discover
 * any serving workstation on the LAN). The active target is the host's
 * perception tier read, re-read on an interval.
 */

import { useEffect, useMemo, useState } from "react";

import {
  ConfigSelectField,
  type ConfigRecord,
  type SetConfigValue,
} from "@altnautica/mapping-ui/components/config/ConfigFields";
import type { SelectOption } from "@altnautica/mapping-ui/components/ui/Select";
import { useHost } from "@altnautica/mapping-ui/host/context";
import { usePairedNodes } from "@altnautica/mapping-ui/hooks/use-paired-nodes";
import { nodeLabelForAddr } from "../../lib/nodes/node-label";
import { NO_DATA_GLYPH } from "@altnautica/mapping-ui/lib/utils";
import { nodeToOffloadAddr } from "../../lib/vision/offload-target";
import { ENABLE_OPTIONS, perceptionT as t } from "./enable-options";

/** How often the active offload target is re-read, in ms. */
const TIER_POLL_MS = 5000;

/** The active target: a `host:port`, null when not offloading, or undefined
 * while unknown (no read landed yet, or the last read was refused). */
type ActiveTarget = string | null | undefined;

export interface OffloadConfigProps {
  config: ConfigRecord | null;
  readOnly: boolean;
  setValue: SetConfigValue;
}

function useActiveOffloadTarget(): ActiveTarget {
  const host = useHost();
  const [target, setTarget] = useState<ActiveTarget>(undefined);
  useEffect(() => {
    let cancelled = false;
    const read = () => {
      host.ctx.perception.readTier().then(
        (info) => {
          if (!cancelled) setTarget(info.offloadTarget);
        },
        () => {
          if (!cancelled) setTarget(undefined);
        },
      );
    };
    read();
    const id = setInterval(read, TIER_POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [host]);
  return target;
}

export function DroneOffloadClient({ config, readOnly, setValue }: OffloadConfigProps) {
  const nodes = usePairedNodes();
  const activeTarget = useActiveOffloadTarget();

  const workstations = useMemo(
    () => nodes.filter((n) => n.profile === "workstation" || n.profile === "compute"),
    [nodes],
  );
  const pinOptions: SelectOption[] = useMemo(() => {
    const options: SelectOption[] = [{ value: "", label: t("offload.pinAuto") }];
    for (const n of workstations) {
      const addr = nodeToOffloadAddr(n);
      if (addr) options.push({ value: addr, label: n.name || n.deviceId });
    }
    return options;
  }, [workstations]);

  const pinHint =
    workstations.length === 0
      ? t("offload.pinNoWorkstation")
      : pinOptions.length === 1
        ? t("offload.pinNoAddress")
        : t("offload.pinHint");

  // The address the host reports, named by the paired node serving it.
  const activeLabel = activeTarget ? nodeLabelForAddr(activeTarget, nodes) : null;

  return (
    <div className="rec:space-y-4">
      {/* Active offload target, as the host reports the perception tier. */}
      <div className="rec:flex rec:items-baseline rec:justify-between rec:gap-3">
        <div className="rec:min-w-0">
          <div className="rec:text-xs rec:text-text-secondary">{t("offload.activeLabel")}</div>
          <p className="rec:mt-0.5 rec:text-[11px] rec:text-text-tertiary">{t("offload.activeHint")}</p>
        </div>
        <div className="rec:shrink-0 rec:font-mono rec:text-sm rec:text-text-primary" title={activeLabel?.title}>
          {activeLabel ? (
            activeLabel.label
          ) : (
            <span className="rec:text-text-tertiary">
              {activeTarget === null ? t("offload.activeNone") : NO_DATA_GLYPH}
            </span>
          )}
        </div>
      </div>

      <ConfigSelectField
        configKey="reconstruct.offload.enabled"
        label={t("offload.enabledLabel")}
        hint={t("offload.enabledHint")}
        options={ENABLE_OPTIONS}
        placeholder={t("enabledAutoDefault")}
        config={config}
        readOnly={readOnly}
        setValue={setValue}
      />

      <ConfigSelectField
        configKey="reconstruct.offload.compute_node_addr"
        label={t("offload.pinLabel")}
        hint={pinHint}
        options={pinOptions}
        placeholder={t("offload.pinAuto")}
        config={config}
        readOnly={readOnly}
        setValue={setValue}
      />
    </div>
  );
}
