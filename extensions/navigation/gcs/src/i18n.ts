import type { PluginContext } from "@altnautica/plugin-sdk";

import en from "../../locales/en.json";

type LocaleTree = { [key: string]: string | LocaleTree };

/**
 * Flatten the shipped locale tree into the dotted keys the host's i18n
 * lookup uses. `locales/en.json` is the single source for this copy:
 * the manifest ships it and the bundle embeds the same file, so the two
 * cannot drift.
 */
function flattenLocale(tree: LocaleTree, prefix: string): Record<string, string> {
  const out: Record<string, string> = {};
  for (const [key, value] of Object.entries(tree)) {
    const path = `${prefix}${key}`;
    if (typeof value === "string") {
      out[path] = value;
    } else {
      Object.assign(out, flattenLocale(value, `${path}.`));
    }
  }
  return out;
}

export const FLAT_LOCALE = flattenLocale(en as LocaleTree, "navigation.");

export type TFn = PluginContext["i18n"]["t"];
export type TrVars = Record<string, string | number>;

/**
 * Translate `key` through the host. A host that has no bundle yet
 * answers with the key itself; the bundled English then stands in, with
 * `{name}` slots filled from `vars` the way the SDK fills them.
 */
export function tr(t: TFn, key: string, vars?: TrVars): string {
  const host = t(key, vars);
  if (host !== key) return host;
  const tpl = FLAT_LOCALE[key];
  if (tpl === undefined) return key;
  if (!vars) return tpl;
  return tpl.replace(/\{(\w+)\}/g, (slot, name: string) =>
    vars[name] === undefined ? slot : String(vars[name]),
  );
}

/** The dash shown for a value the agent has not reported. */
export const DASH = "—";
