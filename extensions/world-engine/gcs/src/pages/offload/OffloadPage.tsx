/**
 * @module pages/offload/OffloadPage
 * @description Where perception executes for the mounted node. A drone gets
 * the offload client (`offload.*`); a workstation or compute node gets the
 * serving controls (`serving.*`) plus its live GPU facts. Every field binds
 * to this extension's config on the mounted node, read back after each write.
 * Any other profile gets a short note.
 */

import { Layers } from "lucide-react";

import { InfoNote, Section, usePluginConfig } from "../../components/config/ConfigFields";
import { useHost } from "../../host/context";
import { DroneOffloadClient } from "./DroneOffloadClient";
import { perceptionT as t } from "./enable-options";
import { WorkstationServing } from "./WorkstationServing";

export function OffloadPage() {
  const { node } = useHost();
  const isDrone = node.profile === "drone";
  const isServer = node.profile === "workstation" || node.profile === "compute";
  const { config, setValue, error } = usePluginConfig(isDrone || isServer ? node.deviceId : null);

  if (!node.deviceId || (!isDrone && !isServer)) {
    return (
      <div className="we:p-4">
        <InfoNote>{t("unsupportedProfile")}</InfoNote>
      </div>
    );
  }

  const readOnly = config === null;
  return (
    <div className="we:p-4">
      <Section
        title={isDrone ? t("offloadTitle") : t("title")}
        icon={Layers}
        blurb={isDrone ? t("offload.blurb") : t("serving.blurb")}
      >
        {error ? <InfoNote>{error}</InfoNote> : null}
        {isDrone ? (
          <DroneOffloadClient config={config} readOnly={readOnly} setValue={setValue} />
        ) : (
          <WorkstationServing config={config} readOnly={readOnly} setValue={setValue} />
        )}
      </Section>
    </div>
  );
}
