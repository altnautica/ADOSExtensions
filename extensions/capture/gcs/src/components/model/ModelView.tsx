/**
 * A drone's Model view: what the paired reconstruction node has built from
 * this drone's sessions, read-only. A session selector and a viewer switcher
 * over the selected reconstruction; measurement and editing stay on the
 * workstation.
 */

import { useState } from "react";
import { Boxes } from "lucide-react";

import { ModelViewport } from "@altnautica/mapping-ui/components/model/ModelViewport";
import { ViewerSwitcher } from "@altnautica/mapping-ui/components/model/ViewerSwitcher";
import { DEFAULT_VIEWER, type ModelViewer } from "@altnautica/mapping-ui/components/model/viewer-types";
import { ViewerLoading } from "@altnautica/mapping-ui/components/model/viewers/ViewerLoading";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";
import { Select } from "@altnautica/mapping-ui/components/ui/Select";
import { useOutputFrame } from "@altnautica/mapping-ui/hooks/use-output-frame";

import { translator } from "../../host/i18n";
import { useNodeModel } from "../../hooks/use-node-model";

const t = translator("setup");

export function ModelView() {
  const [session, setSession] = useState("");
  const [override, setOverride] = useState<{ key: string; viewer: ModelViewer } | null>(null);
  const model = useNodeModel({ sessionId: session || null });
  const frame = useOutputFrame(model.computeClient, model.output, model.outputs);

  if (!model.hasComputeNode) return <CalmState icon={Boxes} message={t("worldModelNoNode")} />;
  if (model.status === "unreachable") return <CalmState icon={Boxes} message={t("modelUnreachable")} />;
  if (model.artifact === null) return <CalmState icon={Boxes} message={t("modelEmpty")} />;

  const selected = model.sessions.find((s) => s.sessionId === session)?.sessionId ?? model.sessions[0]?.sessionId ?? "";
  const viewer = override && override.key === selected ? override.viewer : (model.viewerHint ?? DEFAULT_VIEWER);

  return (
    <div className="cap:flex cap:h-full cap:flex-col">
      <div className="cap:flex cap:items-center cap:gap-2 cap:border-b cap:border-border-default cap:p-3">
        <Select
          options={model.sessions.map((s) => ({ value: s.sessionId, label: s.sessionId.slice(0, 12) }))}
          value={selected}
          onChange={setSession}
          placeholder={t("worldModelSelectSession")}
          className="cap:w-64"
        />
        <ViewerSwitcher
          viewer={viewer}
          onSelect={(v) => setOverride({ key: selected, viewer: v })}
          ariaLabel={t("viewerGroupLabel")}
        />
      </div>
      <div className="cap:relative cap:min-h-[320px] cap:flex-1">
        {frame === undefined ? (
          <ViewerLoading label={t("worldModelHeading")} />
        ) : (
          <ModelViewport viewer={viewer} artifact={model.artifact} backend={model.backend} frame={frame} />
        )}
      </div>
    </div>
  );
}
