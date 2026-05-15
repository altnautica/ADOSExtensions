import type { CSSProperties } from "react";

interface ParamRow {
  id: string;
  description: string;
}

interface ParamGroup {
  title: string;
  rows: ParamRow[];
}

const GROUPS: ParamGroup[] = [
  {
    title: "External vision",
    rows: [
      { id: "EKF2_EV_CTRL", description: "External vision fusion control bitmask." },
      { id: "EKF2_EV_DELAY", description: "External vision measurement delay (ms)." },
    ],
  },
  {
    title: "Optical flow",
    rows: [
      { id: "EKF2_OF_CTRL", description: "Optical flow fusion control bitmask." },
      { id: "SENS_FLOW_MAXR", description: "Maximum flow rate the sensor reports (rad/s)." },
    ],
  },
  {
    title: "Aid + height",
    rows: [
      { id: "EKF2_AID_MASK", description: "Aiding sensor selection bitmask." },
      { id: "EKF2_HGT_REF", description: "Primary height source reference." },
    ],
  },
];

export function Px4VisionParams(): JSX.Element {
  const card: CSSProperties = {
    background: "var(--vn-surface, rgba(255,255,255,0.02))",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.5rem",
    padding: "0.875rem 1rem",
    color: "var(--vn-text, #e5e7eb)",
    display: "flex",
    flexDirection: "column",
    gap: "0.75rem",
  };
  const heading: CSSProperties = {
    fontSize: "0.8125rem",
    fontWeight: 600,
    margin: 0,
    color: "var(--vn-text-2, #cbd5e1)",
  };
  const subheading: CSSProperties = {
    fontSize: "0.75rem",
    fontWeight: 600,
    textTransform: "uppercase",
    letterSpacing: "0.05em",
    color: "var(--vn-text-3, #94a3b8)",
    margin: "0 0 0.375rem 0",
  };
  const note: CSSProperties = {
    fontSize: "0.75rem",
    color: "var(--vn-text-3, #94a3b8)",
    margin: 0,
  };
  const table: CSSProperties = {
    display: "grid",
    gridTemplateColumns: "max-content 1fr",
    columnGap: "0.875rem",
    rowGap: "0.25rem",
    fontSize: "0.8125rem",
  };
  const paramName: CSSProperties = {
    fontFamily: "var(--vn-mono, ui-monospace, SFMono-Regular, monospace)",
    color: "var(--vn-text, #e5e7eb)",
  };
  const paramDesc: CSSProperties = {
    color: "var(--vn-text-3, #94a3b8)",
  };

  return (
    <section style={card} data-testid="vn-px4-params">
      <h3 style={heading}>PX4 vision parameters</h3>
      <p style={note}>
        Read-only summary. Edit in Mission Control&apos;s parameter editor and
        restart the EKF after changes.
      </p>
      {GROUPS.map((group) => (
        <div key={group.title}>
          <p style={subheading}>{group.title}</p>
          <dl style={table}>
            {group.rows.map((row) => (
              <div key={row.id} style={{ display: "contents" }}>
                <dt style={paramName}>{row.id}</dt>
                <dd style={paramDesc}>{row.description}</dd>
              </div>
            ))}
          </dl>
        </div>
      ))}
    </section>
  );
}
