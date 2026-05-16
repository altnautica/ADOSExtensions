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
    title: "Optical flow source",
    rows: [
      {
        id: "opflow_hardware",
        description:
          "Optical flow driver. Set to MAVLINK to consume the plugin's OPTICAL_FLOW_RAD messages over the FC's MAVLink rx UART.",
      },
      {
        id: "opflow_scale",
        description:
          "Pixels-per-radian scale. Leave default when using the MAVLINK driver; the plugin sends pre-scaled angular rates.",
      },
    ],
  },
  {
    title: "Position hold + navigation",
    rows: [
      {
        id: "nav_use_optflow_for_poshold",
        description:
          "Set to ON. Enables position hold (NAV POSHOLD mode) using the optical-flow stream when GPS is unavailable.",
      },
      {
        id: "nav_extra_arming_safety",
        description:
          "When ON the FC refuses to arm until flow quality is reported as healthy. Recommended for indoor and over-ground flight.",
      },
      {
        id: "nav_landing_speed",
        description:
          "Vertical speed during NAV LAND. Tune so landings stay inside the rangefinder's valid altitude band.",
      },
    ],
  },
  {
    title: "Rangefinder",
    rows: [
      {
        id: "rangefinder_hardware",
        description:
          "Set to the wired driver (LIDARMT for MTF-01, VL53L1X, TFMINI, etc.) or MSP to relay a companion-side reading.",
      },
      {
        id: "nav_rangefinder_for_terrain",
        description:
          "Set to ON when the operator wants terrain-following behavior on top of flow-based position hold.",
      },
    ],
  },
];

const NOTE_LINES: string[] = [
  "iNav 7.0 or newer is required for OPTICAL_FLOW_RAD reception over MAVLink rx.",
  "Set the FC's serial port function to MAVLINK on the UART wired to the ADOS Drone Agent.",
  "The plugin's component id is 198. iNav accepts the message from any component when opflow_hardware = MAVLINK.",
  "VIO modes are not surfaced on iNav in this release; the plugin disables them in the mode picker.",
];

export function InavVisionParams(): JSX.Element {
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
  const noteList: CSSProperties = {
    fontSize: "0.75rem",
    color: "var(--vn-text-3, #94a3b8)",
    margin: 0,
    paddingLeft: "1.25rem",
  };

  return (
    <section style={card} data-testid="vn-inav-params">
      <h3 style={heading}>iNav optical-flow parameters</h3>
      <p style={note}>
        Read-only summary. Edit in Mission Control&apos;s parameter
        editor over MSP.
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
      <div>
        <p style={subheading}>Setup notes</p>
        <ul style={noteList}>
          {NOTE_LINES.map((line) => (
            <li key={line}>{line}</li>
          ))}
        </ul>
      </div>
    </section>
  );
}
