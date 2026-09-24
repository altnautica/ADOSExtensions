/**
 * The extension's own strings (`locales/en.json`), looked up by dotted key
 * under a namespace. Messages interpolate `{name}` placeholders and support
 * the one ICU form the catalogue uses, `{n, plural, one {...} other {...}}`
 * with `#` standing for the number. A missing key renders as the key itself.
 */

import en from "../../../locales/en.json";

/** The one locale the extension ships (number and date formatting). */
export const LOCALE = "en";

export type TranslateParams = Record<string, string | number>;
export type Translate = (key: string, params?: TranslateParams) => string;

type Catalogue = { [key: string]: string | Catalogue };

const CATALOGUE: Catalogue = en;

function lookup(path: string): string | null {
  let node: string | Catalogue | undefined = CATALOGUE;
  for (const part of path.split(".")) {
    if (typeof node !== "object") return null;
    node = node[part];
  }
  return typeof node === "string" ? node : null;
}

/** Resolve the plain placeholders (inside plural arms too), then the plural
 * blocks. A placeholder with no matching param is left as written. */
export function formatMessage(message: string, params: TranslateParams = {}): string {
  const plain = message.replace(/\{(\w+)\}/g, (whole, name: string) => {
    const value = params[name];
    return value === undefined ? whole : String(value);
  });
  return plain.replace(
    /\{(\w+),\s*plural,\s*((?:(?:=\d+|\w+)\s*\{[^{}]*\}\s*)+)\}/g,
    (whole, name: string, arms: string) => {
      const value = params[name];
      if (typeof value !== "number") return whole;
      const choices = new Map<string, string>();
      for (const m of arms.matchAll(/(=\d+|\w+)\s*\{([^{}]*)\}/g)) choices.set(m[1] ?? "", m[2] ?? "");
      const arm =
        choices.get(`=${value}`) ?? choices.get(value === 1 ? "one" : "other") ?? choices.get("other");
      return arm === undefined ? whole : arm.replace(/#/g, String(value));
    },
  );
}

/** A translator bound to one namespace (`"atlas"`, `"nodeSettings.atlas"`). */
export function translator(namespace: string): Translate {
  return (key, params) => {
    const full = `${namespace}.${key}`;
    const message = lookup(full);
    return message === null ? full : formatMessage(message, params);
  };
}
