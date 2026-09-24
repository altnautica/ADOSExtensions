/**
 * @module JobsPanel
 * @description A workstation's Jobs view: one list over the compute node's
 * reconstruction / offload jobs, grouped Flat / By dataset / By status, with a
 * control to queue a new reconstruction. Calm states when the node has no
 * agent or its job API is unreachable, never an error.
 */

import { useMemo, useState, type ReactNode } from "react";
import { Boxes } from "lucide-react";

import { Select, type SelectOption } from "../../components/ui/Select";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "../../hooks/use-compute-jobs";
import type { ComputeJob } from "../../lib/agent/compute-client";
import { CalmState } from "./CalmState";
import { ForgeJobs, JOB_STATE_KEYS } from "./ForgeJobs";
import { SubmitJobButton } from "./SubmitJobButton";

const t = translator("atlas");
const tNode = translator("nodeConsole");

type GroupBy = "flat" | "dataset" | "status";

/** Group-by option → its `nodeConsole.jobs.*` key. */
const GROUP_LABEL_KEYS: Record<GroupBy, string> = {
  flat: "jobs.flat",
  dataset: "jobs.byDataset",
  status: "jobs.byStatus",
};

const GROUP_OPTIONS: readonly SelectOption[] = (["flat", "dataset", "status"] as const).map((g) => ({
  value: g,
  label: tNode(GROUP_LABEL_KEYS[g]),
}));

function isGroupBy(v: string): v is GroupBy {
  return Object.hasOwn(GROUP_LABEL_KEYS, v);
}

/** Preferred ordering of the "By status" bands; unknown states trail after. */
const STATUS_ORDER = ["queued", "running", "completed", "failed", "cancelled"];

/** Bucket key for jobs that name no dataset. */
const NO_DATASET = "__none__";

/** Group a job list by a derived key, preserving first-seen order. */
function bucket(list: readonly ComputeJob[], keyOf: (j: ComputeJob) => string): Map<string, ComputeJob[]> {
  const map = new Map<string, ComputeJob[]>();
  for (const j of list) {
    const k = keyOf(j);
    const arr = map.get(k);
    if (arr) arr.push(j);
    else map.set(k, [j]);
  }
  return map;
}

function groupJobs(sorted: readonly ComputeJob[], groupBy: GroupBy) {
  if (groupBy === "dataset") {
    return [...bucket(sorted, (j) => j.datasetId ?? NO_DATASET)].map(([key, jobs]) => ({
      key,
      label: key === NO_DATASET ? tNode("jobs.noDataset") : key,
      jobs,
    }));
  }
  const map = bucket(sorted, (j) => j.state);
  const ordered = [
    ...STATUS_ORDER.filter((s) => map.has(s)),
    ...[...map.keys()].filter((s) => !STATUS_ORDER.includes(s)),
  ];
  return ordered.map((state) => {
    const key = JOB_STATE_KEYS[state];
    return { key: state, label: key ? t(key) : state, jobs: map.get(state) ?? [] };
  });
}

function GroupHeader({ label, count }: { label: string; count: number }) {
  return (
    <div className="we:flex we:items-center we:gap-1.5 we:px-2 we:pb-1 we:pt-2 we:text-[10px] we:uppercase we:tracking-wide we:text-text-tertiary">
      <span className="we:truncate">{label}</span>
      <span className="we:font-mono we:tabular-nums">· {count}</span>
    </div>
  );
}

export function JobsPanel({ deviceId }: { deviceId: string | null }) {
  const { jobs, loading, unreachable, client } = useComputeJobs(deviceId);
  const [groupBy, setGroupBy] = useState<GroupBy>("flat");

  // Newest first: the Flat order and the input to grouping.
  const sorted = useMemo(() => [...jobs].sort((a, b) => b.createdMs - a.createdMs), [jobs]);

  // Every dataset the engine knows about reached it through a job, so the
  // dataset list derives from the job list rather than a second fetch.
  const datasetIds = useMemo(() => {
    const seen = new Set<string>();
    for (const job of sorted) if (job.datasetId) seen.add(job.datasetId);
    return [...seen];
  }, [sorted]);

  const groups = useMemo(() => (groupBy === "flat" ? [] : groupJobs(sorted, groupBy)), [groupBy, sorted]);

  if (!client) return <CalmState icon={Boxes} message={t("forgeLocalOnly")} />;

  let table: ReactNode;
  if (loading) {
    table = (
      <div className="we:flex we:items-center we:justify-center we:py-16">
        <div className="we:h-5 we:w-5 we:animate-spin we:rounded-full we:border-2 we:border-accent-primary we:border-t-transparent" />
      </div>
    );
  } else if (unreachable) {
    table = <CalmState icon={Boxes} message={t("forgeAwaiting")} />;
  } else if (groupBy === "flat") {
    table = <ForgeJobs jobs={sorted} client={client} />;
  } else {
    table = (
      <div className="we:p-1">
        {groups.map((g) => (
          <div key={g.key}>
            <GroupHeader label={g.label} count={g.jobs.length} />
            <ForgeJobs jobs={g.jobs} client={client} />
          </div>
        ))}
      </div>
    );
  }

  return (
    <div className="we:flex we:h-full we:flex-col">
      <div className="we:flex we:items-center we:gap-2 we:border-b we:border-border-default we:p-2">
        <span className="we:text-[11px] we:text-text-tertiary">{tNode("jobs.groupBy")}</span>
        <Select
          options={GROUP_OPTIONS}
          value={groupBy}
          onChange={(v) => {
            if (isGroupBy(v)) setGroupBy(v);
          }}
          className="we:w-40"
        />
        {/* The dataset list derives from the job list, so it is unknown until
            the first poll lands and while the node is unreachable. */}
        {!loading && !unreachable && (
          <div className="we:ml-auto">
            <SubmitJobButton client={client} datasetIds={datasetIds} jobs={jobs} />
          </div>
        )}
      </div>
      <div className="we:flex-1 we:overflow-y-auto">{table}</div>
    </div>
  );
}
