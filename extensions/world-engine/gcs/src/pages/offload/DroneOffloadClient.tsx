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
} from "../../components/config/ConfigFields";
import type { SelectOption } from "../../components/ui/Select";
import { useHost } from "../../host/context";
import { usePairedNodes } from "../../hooks/use-paired-nodes";
import { NO_DATA_GLYPH } from "../../lib/utils";
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

  return (
    <div className="we:space-y-4">
      {/* Active offload target, as the host reports the perception tier. */}
      <div className="we:flex we:items-baseline we:justify-between we:gap-3">
        <div className="we:min-w-0">
          <div className="we:text-xs we:text-text-secondary">{t("offload.activeLabel")}</div>
          <p className="we:mt-0.5 we:text-[11px] we:text-text-tertiary">{t("offload.activeHint")}</p>
        </div>
        <div className="we:shrink-0 we:font-mono we:text-sm we:text-text-primary">
          {activeTarget ? (
            activeTarget
          ) : (
            <span className="we:text-text-tertiary">
              {activeTarget === null ? t("offload.activeNone") : NO_DATA_GLYPH}
            </span>
          )}
        </div>
      </div>

      <ConfigSelectField
        configKey="offload.enabled"
        label={t("offload.enabledLabel")}
        hint={t("offload.enabledHint")}
        options={ENABLE_OPTIONS}
        placeholder={t("enabledAutoDefault")}
        config={config}
        readOnly={readOnly}
        setValue={setValue}
      />

      <ConfigSelectField
        configKey="offload.compute_node_addr"
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
