import { useRef, type CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

interface Props {
  label: string;
  /** Accept attribute on the underlying <input>. Defaults to ``*``. */
  accept?: string;
  /** Called with the picked file. The picker resets after the call
   * so the same file can be re-picked on retry. */
  onPick: (file: File) => void;
  disabled?: boolean;
  /** Optional testid suffix; renders as ``ext-ui-file-picker-<suffix>``. */
  testIdSuffix?: string;
}

/**
 * Accessible file picker. A visible button + a hidden <input
 * type="file"> kept in sync via a ref. Pressing the button opens the
 * native picker; selecting a file fires ``onPick`` and resets the
 * input so re-picking the same file works.
 */
export function FilePicker({
  label,
  accept = "*",
  onPick,
  disabled = false,
  testIdSuffix,
}: Props): JSX.Element {
  const inputRef = useRef<HTMLInputElement | null>(null);
  const tid = testIdSuffix
    ? `ext-ui-file-picker-${testIdSuffix}`
    : "ext-ui-file-picker";
  return (
    <>
      <button
        type="button"
        style={btn(disabled)}
        onClick={() => {
          if (disabled) return;
          inputRef.current?.click();
        }}
        disabled={disabled}
        data-testid={tid}
      >
        {label}
      </button>
      <input
        ref={inputRef}
        type="file"
        accept={accept}
        style={{ display: "none" }}
        data-testid={`${tid}-input`}
        onChange={(event) => {
          const file = event.target.files?.[0];
          if (file) {
            onPick(file);
          }
          event.target.value = "";
        }}
      />
    </>
  );
}

const btn = (disabled: boolean): CSSProperties => ({
  padding: "0.375rem 0.75rem",
  background: disabled ? TOKENS.surface2 : TOKENS.accent,
  color: disabled ? TOKENS.textMuted : "white",
  border: "none",
  borderRadius: "0.375rem",
  fontWeight: 600,
  cursor: disabled ? "not-allowed" : "pointer",
  fontSize: "0.75rem",
  opacity: disabled ? 0.7 : 1,
});
