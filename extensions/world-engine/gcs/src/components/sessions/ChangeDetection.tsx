/**
 * @module sessions/ChangeDetection
 * @description Compare two captures of one site: pick the earlier and the
 * later session (both uploaded to the same node) and submit a node `change`
 * job, which colours every point that moved more than 5 cm.
 */

import { useState } from "react";
import { GitCompare } from "lucide-react";

import { translator } from "../../host/i18n";
import type { ComputeAgentClient } from "../../lib/agent/compute-client";
import type { CaptureSession } from "../../lib/capture/sessions";
import { Button } from "../ui/Button";
import { Select } from "../ui/Select";
import { useToast } from "../ui/Toast";

const t = translator("sessions");

export function ChangeDetection({
  sessions,
  nodeClient,
}: {
  /** Sessions committed on `nodeClient`'s node. */
  sessions: readonly CaptureSession[];
  nodeClient: ComputeAgentClient | null;
}) {
  const { toast } = useToast();
  const [a, setA] = useState("");
  const [b, setB] = useState("");
  const [busy, setBusy] = useState(false);
  const options = sessions.map((s) => ({ value: s.sessionId, label: s.sessionId }));

  if (!nodeClient || sessions.length < 2) {
    return <p className="we:text-[11px] we:text-text-tertiary">{t("changeNeedsTwo")}</p>;
  }

  return (
    <div className="we:flex we:flex-wrap we:items-end we:gap-2">
      <Select label={t("changeBefore")} options={options} value={a} onChange={setA} placeholder={t("pickSession")} className="we:w-64" />
      <Select label={t("changeAfter")} options={options} value={b} onChange={setB} placeholder={t("pickSession")} className="we:w-64" />
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
