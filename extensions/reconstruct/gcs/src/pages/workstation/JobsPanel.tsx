/**
 * @module JobsPanel
 * @description A workstation's Jobs view: one list over the compute node's
 * reconstruction / offload jobs, grouped Flat / By dataset / By status, with a
 * control to queue a new reconstruction. Calm states when the node has no
 * agent or its job API is unreachable, never an error.
 */

import { useMemo, useState, type ReactNode } from "react";
import { Boxes } from "lucide-react";

import { Select, type SelectOption } from "@altnautica/mapping-ui/components/ui/Select";
import { translator } from "../../host/i18n";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";
import type { ComputeJob } from "@altnautica/mapping-ui/lib/agent/compute-client";
import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";
import { JobsList, JOB_STATE_KEYS } from "./JobsList";
import { SubmitJobButton } from "./SubmitJobButton";

const t = translator("node");
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
    <div className="rec:flex rec:items-center rec:gap-1.5 rec:px-2 rec:pb-1 rec:pt-2 rec:text-[10px] rec:uppercase rec:tracking-wide rec:text-text-tertiary">
      <span className="rec:truncate">{label}</span>
      <span className="rec:font-mono rec:tabular-nums">· {count}</span>
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
      <div className="rec:flex rec:items-center rec:justify-center rec:py-16">
        <div className="rec:h-5 rec:w-5 rec:animate-spin rec:rounded-full rec:border-2 rec:border-accent-primary rec:border-t-transparent" />
      </div>
    );
  } else if (unreachable) {
    table = <CalmState icon={Boxes} message={t("forgeAwaiting")} />;
  } else if (groupBy === "flat") {
    table = <JobsList jobs={sorted} client={client} />;
  } else {
    table = (
      <div className="rec:p-1">
        {groups.map((g) => (
          <div key={g.key}>
            <GroupHeader label={g.label} count={g.jobs.length} />
            <JobsList jobs={g.jobs} client={client} />
          </div>
        ))}
      </div>
    );
  }

  return (
    <div className="rec:flex rec:h-full rec:flex-col">
      <div className="rec:flex rec:items-center rec:gap-2 rec:border-b rec:border-border-default rec:p-2">
        <span className="rec:text-[11px] rec:text-text-tertiary">{tNode("jobs.groupBy")}</span>
        <Select
          options={GROUP_OPTIONS}
          value={groupBy}
          onChange={(v) => {
            if (isGroupBy(v)) setGroupBy(v);
          }}
          className="rec:w-40"
        />
        {/* The dataset list derives from the job list, so it is unknown until
            the first poll lands and while the node is unreachable. */}
        {!loading && !unreachable && (
          <div className="rec:ml-auto">
            <SubmitJobButton client={client} datasetIds={datasetIds} jobs={jobs} />
          </div>
        )}
      </div>
      <div className="rec:flex-1 rec:overflow-y-auto">{table}</div>
    </div>
  );
}
