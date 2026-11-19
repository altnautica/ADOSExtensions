/**
 * @module editor/Toolbar
 * @description The editor's one toolbar: Select · Box · Polygon · Crop ·
 * Delete · Patch · Decimate · Measure · Top view · Undo, with the hint line
 * under it. Mode tools show their state with `aria-pressed`; tool switches do
 * not animate (they are keyboard-frequency).
 */

import type { ReactNode } from "react";
import {
  Crop,
  Eraser,
  Grid2x2,
  Layers2,
  Minimize2,
  MousePointer2,
  PenTool,
  Ruler,
  Square,
  Undo2,
} from "lucide-react";

import { cn } from "@altnautica/mapping-ui/lib/utils";

import { translator } from "../../host/i18n";
import type { Mode } from "./editor-state";

const t = translator("editor");

function ToolButton({
  label,
  icon,
  pressed,
  disabled,
  onClick,
}: {
  label: string;
  icon: ReactNode;
  pressed?: boolean;
  disabled?: boolean;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      aria-pressed={pressed}
      disabled={disabled}
      onClick={onClick}
      title={label}
      className={cn(
        "rec:flex rec:items-center rec:gap-1 rec:rounded rec:px-2 rec:py-1 rec:text-xs",
        pressed ? "rec:bg-accent-primary/15 rec:text-accent-primary" : "rec:text-text-secondary rec:hover:bg-bg-tertiary",
        disabled && "rec:opacity-40",
      )}
    >
      {icon}
      <span>{label}</span>
    </button>
  );
}

export interface ToolbarProps {
  mode: Mode;
  view: "top" | "3d";
  readOnly: boolean;
  hasSelection: boolean;
  canUndo: boolean;
  hint: string;
  refusal: string | null;
  onMode: (m: Mode) => void;
  onCrop: () => void;
  onDelete: () => void;
  onDecimate: () => void;
  onView: () => void;
  onUndo: () => void;
}

export function Toolbar(p: ToolbarProps) {
  const icon = { size: 13, "aria-hidden": true } as const;
  return (
    <div className="rec:border-b rec:border-border-default">
      <div role="toolbar" aria-label={t("toolbar")} className="rec:flex rec:flex-wrap rec:items-center rec:gap-1 rec:px-2 rec:py-1">
        <ToolButton label={t("tool.select")} icon={<MousePointer2 {...icon} />} pressed={p.mode === "select"} onClick={() => p.onMode("select")} />
        <ToolButton label={t("tool.box")} icon={<Square {...icon} />} pressed={p.mode === "box"} disabled={p.readOnly} onClick={() => p.onMode("box")} />
        <ToolButton label={t("tool.polygon")} icon={<PenTool {...icon} />} pressed={p.mode === "polygon"} disabled={p.readOnly} onClick={() => p.onMode("polygon")} />
        <ToolButton label={t("tool.crop")} icon={<Crop {...icon} />} disabled={p.readOnly || !p.hasSelection} onClick={p.onCrop} />
        <ToolButton label={t("tool.delete")} icon={<Eraser {...icon} />} disabled={p.readOnly || !p.hasSelection} onClick={p.onDelete} />
        <ToolButton label={t("tool.patch")} icon={<Layers2 {...icon} />} pressed={p.mode === "patch"} disabled={p.readOnly} onClick={() => p.onMode("patch")} />
        <ToolButton label={t("tool.decimate")} icon={<Minimize2 {...icon} />} disabled={p.readOnly} onClick={p.onDecimate} />
        <ToolButton label={t("tool.measure")} icon={<Ruler {...icon} />} pressed={p.mode === "measure"} onClick={() => p.onMode("measure")} />
        <ToolButton label={t("tool.topView")} icon={<Grid2x2 {...icon} />} pressed={p.view === "top"} onClick={p.onView} />
        <ToolButton label={t("tool.undo")} icon={<Undo2 {...icon} />} disabled={!p.canUndo} onClick={p.onUndo} />
      </div>
      <p
        role="status"
        className={cn("rec:px-3 rec:pb-1 rec:text-[11px]", p.refusal ? "rec:text-status-warning" : "rec:text-text-tertiary")}
      >
        {p.refusal ?? p.hint}
      </p>
    </div>
  );
}
