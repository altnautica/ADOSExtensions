/**
 * Config-bound field primitives for this extension's per-node plugin config.
 * Each field reads its current value from the loaded config by dotted key and
 * writes back through the shared `setValue`. A select writes on change; a text
 * field writes on blur or Enter; a read-only row shows a value another surface
 * owns. The section shell, read rows and notes give every settings page one
 * look.
 */

import { useCallback, useEffect, useState, type KeyboardEvent, type ReactNode } from "react";
import type { LucideIcon } from "lucide-react";

import { useHost } from "../../host/context";
import { translator } from "../../host/i18n";
import { useToast } from "../ui/Toast";
import { Select, type SelectOption } from "../ui/Select";

const t = translator("nodeSettings");

export type ConfigRecord = Record<string, unknown>;
export type SetConfigValue = (key: string, value: unknown) => Promise<void>;

/**
 * Read `dottedKey` (e.g. `atlas.pose_tier`) out of a config that may hold it
 * nested (`{atlas: {pose_tier}}`), flat (`{"atlas.pose_tier": …}`) or any mix
 * of the two. Undefined when absent, so a surface can say "not set" instead of
 * inventing a default.
 */
export function readConfigValue(config: ConfigRecord | null, dottedKey: string): unknown {
  if (!config) return undefined;
  if (dottedKey in config) return config[dottedKey];
  const parts = dottedKey.split(".");
  for (let i = parts.length - 1; i >= 1; i--) {
    const head = parts.slice(0, i).join(".");
    const inner = config[head];
    if (typeof inner === "object" && inner !== null && !Array.isArray(inner)) {
      const found = readConfigValue(inner as ConfigRecord, parts.slice(i).join("."));
      if (found !== undefined) return found;
    }
  }
  return undefined;
}

/** A config value as display text, or "" when unset. */
function configText(raw: unknown): string {
  if (typeof raw === "string") return raw;
  return raw == null ? "" : String(raw);
}

export interface PluginConfigState {
  /** The node's plugin config, or null until the first read lands. */
  config: ConfigRecord | null;
  /** Write one key, then re-read so `config` holds the persisted value.
   * Rejects when the write fails. */
  setValue: SetConfigValue;
  /** Why the last read failed, or null. */
  error: string | null;
}

/**
 * This extension's plugin config on `deviceId`: read once through the node's
 * agent, kept current by the host's config push when `deviceId` is the mounted
 * node, and re-read after every write.
 */
export function usePluginConfig(deviceId: string | null): PluginConfigState {
  const host = useHost();
  const [state, setState] = useState<{ deviceId: string; config: ConfigRecord | null; error: string | null } | null>(
    null,
  );

  const reload = useCallback(
    async (id: string): Promise<void> => {
      try {
        const next = await host.nodes.pluginConfig(id).get();
        setState({ deviceId: id, config: next, error: null });
      } catch (err) {
        setState((prev) => ({
          deviceId: id,
          config: prev && prev.deviceId === id ? prev.config : null,
          error: err instanceof Error ? err.message : String(err),
        }));
      }
    },
    [host],
  );

  useEffect(() => {
    if (!deviceId) return;
    let cancelled = false;
    void reload(deviceId);
    const unsubscribe =
      deviceId === host.node.deviceId
        ? host.ctx.config.onChange<unknown>((next) => {
            if (!cancelled && typeof next === "object" && next !== null && !Array.isArray(next)) {
              setState({ deviceId, config: next as ConfigRecord, error: null });
            }
          })
        : null;
    return () => {
      cancelled = true;
      unsubscribe?.();
    };
  }, [host, deviceId, reload]);

  const setValue = useCallback<SetConfigValue>(
    async (key, value) => {
      if (!deviceId) throw new Error(t("applyFailed"));
      await host.nodes.pluginConfig(deviceId).set(key, value);
      await reload(deviceId);
    },
    [host, deviceId, reload],
  );

  const current = state && state.deviceId === deviceId ? state : null;
  return { config: current?.config ?? null, setValue, error: current?.error ?? null };
}

interface BaseProps {
  configKey: string;
  label: string;
  hint?: string;
  config: ConfigRecord | null;
  readOnly: boolean;
  setValue: SetConfigValue;
}

/** A select bound to a string config key; writes on change. A stored value
 * the option list does not carry is added as its own option, so it never
 * reads as the placeholder. */
export function ConfigSelectField({
  configKey,
  label,
  hint,
  options,
  placeholder,
  config,
  readOnly,
  setValue,
}: BaseProps & { options: readonly SelectOption[]; placeholder?: string }) {
  const { toast } = useToast();
  const [pending, setPending] = useState<string | null>(null);
  const current = configText(readConfigValue(config, configKey));
  const value = pending ?? current;
  const shownOptions =
    current !== "" && !options.some((o) => o.value === current)
      ? [...options, { value: current, label: current }]
      : options;

  const onChange = async (next: string) => {
    if (readOnly || pending !== null || next === value) return;
    setPending(next);
    try {
      await setValue(configKey, next);
      toast(t("applied"), "success");
    } catch (err) {
      toast(err instanceof Error ? err.message : t("applyFailed"), "error");
    } finally {
      setPending(null);
    }
  };

  return (
    <div className="we:flex we:flex-col we:gap-1.5">
      <Select
        label={label}
        options={shownOptions}
        value={value}
        onChange={(v) => void onChange(v)}
        disabled={readOnly || pending !== null}
        placeholder={placeholder ?? t("notSet")}
      />
      {hint ? <p className="we:text-[11px] we:text-text-tertiary">{hint}</p> : null}
    </div>
  );
}

/** A text input bound to a string config key; writes on blur or Enter when the
 * draft differs from the stored value. An empty draft writes an empty string. */
export function ConfigTextField({
  configKey,
  label,
  hint,
  placeholder,
  config,
  readOnly,
  setValue,
}: BaseProps & { placeholder?: string }) {
  const { toast } = useToast();
  const current = configText(readConfigValue(config, configKey));
  const [draft, setDraft] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);
  const value = draft ?? current;

  const commit = async () => {
    if (readOnly || saving || draft === null) return;
    if (draft === current) {
      setDraft(null);
      return;
    }
    setSaving(true);
    try {
      await setValue(configKey, draft);
      toast(t("applied"), "success");
      setDraft(null);
    } catch (err) {
      toast(err instanceof Error ? err.message : t("applyFailed"), "error");
    } finally {
      setSaving(false);
    }
  };

  const onKeyDown = (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter") void commit();
    if (e.key === "Escape") setDraft(null);
  };

  return (
    <label className="we:flex we:flex-col we:gap-1.5">
      <span className="we:text-[11px] we:text-text-secondary">{label}</span>
      <input
        type="text"
        value={value}
        placeholder={placeholder}
        disabled={readOnly || saving}
        onChange={(e) => setDraft(e.target.value)}
        onBlur={() => void commit()}
        onKeyDown={onKeyDown}
        className="we:h-8 we:w-full we:rounded we:border we:border-border-default we:bg-bg-tertiary we:px-2.5 we:font-mono we:text-xs we:text-text-primary we:outline-none we:focus:border-accent-primary we:disabled:cursor-not-allowed we:disabled:opacity-50"
      />
      {hint ? <span className="we:text-[11px] we:text-text-tertiary">{hint}</span> : null}
    </label>
  );
}

/** A labelled read-only config value another surface manages. Shows the real
 * value, or "not set". */
export function ConfigReadonlyRow({
  configKey,
  label,
  config,
  format,
}: {
  configKey: string;
  label: string;
  config: ConfigRecord | null;
  format?: (raw: unknown) => string | null;
}) {
  const raw = readConfigValue(config, configKey);
  const text = configText(raw);
  const shown = format ? format(raw) : text.length > 0 ? text : null;
  return (
    <div className="we:flex we:items-baseline we:justify-between we:gap-3">
      <div className="we:min-w-0 we:text-xs we:text-text-secondary">{label}</div>
      <div className="we:shrink-0 we:font-mono we:text-sm we:text-text-primary">
        {shown ?? <span className="we:text-text-tertiary">{t("notSet")}</span>}
      </div>
    </div>
  );
}

/** A titled settings card with an optional icon and blurb. */
export function Section({
  title,
  icon: Icon,
  blurb,
  children,
}: {
  title: string;
  icon?: LucideIcon;
  blurb?: string;
  children: ReactNode;
}) {
  return (
    <section className="we:rounded we:border we:border-border-default we:bg-bg-secondary we:p-5">
      <div className="we:mb-3 we:flex we:items-center we:gap-2">
        {Icon ? <Icon size={16} className="we:text-accent-primary" aria-hidden="true" /> : null}
        <h2 className="we:text-lg we:font-medium we:text-text-primary">{title}</h2>
      </div>
      {blurb ? <p className="we:mb-4 we:text-xs we:text-text-secondary">{blurb}</p> : null}
      <div className="we:space-y-4">{children}</div>
    </section>
  );
}

/** A label / mono value pair for a reported fact; a null or empty value says
 * "not reported" rather than rendering a blank. */
export function ReadRow({ label, value }: { label: string; value: string | null }) {
  return (
    <div className="we:flex we:items-baseline we:justify-between we:gap-3">
      <span className="we:text-[11px] we:text-text-tertiary">{label}</span>
      <span className="we:min-w-0 we:truncate we:text-right we:font-mono we:text-xs we:text-text-primary">
        {value != null && value.length > 0 ? (
          value
        ) : (
          <span className="we:text-text-tertiary">{t("notReported")}</span>
        )}
      </span>
    </div>
  );
}

/** A muted explanatory note. */
export function InfoNote({ children }: { children: ReactNode }) {
  return (
    <div className="we:rounded we:border we:border-border-default/60 we:bg-bg-tertiary/40 we:px-3 we:py-2 we:text-[11px] we:text-text-tertiary">
      {children}
    </div>
  );
}
