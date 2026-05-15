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
    title: "Sensor drivers",
    rows: [
      { id: "VISO_TYPE", description: "Vision odometry backend selection." },
      { id: "FLOW_TYPE", description: "Optical flow sensor backend (5 = MAVLink)." },
    ],
  },
  {
    title: "EKF source sets",
    rows: [
      { id: "EK3_SRC1_POSXY", description: "Source set 1 horizontal position." },
      { id: "EK3_SRC1_VELXY", description: "Source set 1 horizontal velocity." },
      { id: "EK3_SRC2_POSXY", description: "Source set 2 horizontal position." },
      { id: "EK3_SRC2_VELXY", description: "Source set 2 horizontal velocity." },
      { id: "EK3_SRC3_POSXY", description: "Source set 3 horizontal position." },
      { id: "EK3_SRC3_VELXY", description: "Source set 3 horizontal velocity." },
      { id: "EK3_SRC_OPTIONS", description: "Source-set switching options bitmask." },
    ],
  },
  {
    title: "Flow tuning",
    rows: [
      { id: "EK3_FLOW_DELAY", description: "Optical flow measurement delay (ms)." },
      { id: "EK3_FLOW_QUAL_MIN", description: "Minimum flow quality (0-255)." },
      { id: "FLOW_HEIGHT_MIN", description: "Minimum AGL for flow fusion (m)." },
      { id: "FLOW_HEIGHT_MAX", description: "Maximum AGL for flow fusion (m)." },
    ],
  },
];

export function ArduPilotVisionParams(): JSX.Element {
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
    <section style={card} data-testid="vn-ardupilot-params">
      <h3 style={heading}>ArduPilot vision parameters</h3>
      <p style={note}>
        Read-only summary. Edit in Mission Control&apos;s parameter editor.
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
