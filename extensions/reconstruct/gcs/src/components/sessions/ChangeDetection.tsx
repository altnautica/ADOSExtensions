/**
 * @module sessions/ChangeDetection
 * @description Compare two captures of one site: pick the earlier and the
 * later session (both uploaded to the same node) and submit a node `change`
 * job, which colours every point that moved more than 5 cm.
 */

import { useState } from "react";
import { GitCompare } from "lucide-react";

import { translator } from "../../host/i18n";
import type { ComputeAgentClient } from "@altnautica/mapping-ui/lib/agent/compute-client";
import { Button } from "@altnautica/mapping-ui/components/ui/Button";
import { Select } from "@altnautica/mapping-ui/components/ui/Select";
import { useToast } from "@altnautica/mapping-ui/components/ui/Toast";

const t = translator("sessions");

export function ChangeDetection({
  sessionIds,
  nodeClient,
}: {
  /** Ids of sessions uploaded to `nodeClient`'s node. */
  sessionIds: readonly string[];
  nodeClient: ComputeAgentClient | null;
}) {
  const { toast } = useToast();
  const [a, setA] = useState("");
  const [b, setB] = useState("");
  const [busy, setBusy] = useState(false);
  const options = sessionIds.map((id) => ({ value: id, label: id }));

  if (!nodeClient || sessionIds.length < 2) {
    return <p className="rec:text-[11px] rec:text-text-tertiary">{t("changeNeedsTwo")}</p>;
  }

  return (
    <div className="rec:flex rec:flex-wrap rec:items-end rec:gap-2">
      <Select label={t("changeBefore")} options={options} value={a} onChange={setA} placeholder={t("pickSession")} className="rec:w-64" />
      <Select label={t("changeAfter")} options={options} value={b} onChange={setB} placeholder={t("pickSession")} className="rec:w-64" />
      <Button
        icon={<GitCompare size={12} />}
        loading={busy}
        disabled={!a || !b || a === b}
        onClick={async () => {
          setBusy(true);
          const res = await nodeClient.submit("change", { a, b });
          setBusy(false);
          if (res.ok) toast(t("changeQueued"), "success");
          else toast(res.message, "error");
        }}
      >
        {t("compare")}
      </Button>
    </div>
  );
}
