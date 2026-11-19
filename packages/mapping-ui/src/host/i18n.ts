/**
 * Message catalogues for the mapping extensions: dotted keys under a
 * namespace, `{name}` placeholders, and the one ICU form the catalogues use,
 * `{n, plural, one {...} other {...}}` with `#` standing for the number. A
 * missing key renders as the key itself.
 *
 * The package's own strings (the viewers') live in `locales/en.json` beside
 * it; each extension builds its translator over its own catalogue with
 * {@link catalogueTranslator}.
 */

import en from "../../locales/en.json";

/** The one locale the extensions ship (number and date formatting). The
 * host's `ctx.i18n` resolves keys against a bundle but does not expose which
 * locale the operator selected, so there is nothing to pick a catalogue by. */
export const LOCALE = "en";

export type TranslateParams = Record<string, string | number>;
export type Translate = (key: string, params?: TranslateParams) => string;

export type Catalogue = { [key: string]: string | Catalogue };

function lookup(catalogue: Catalogue, path: string): string | null {
  let node: string | Catalogue | undefined = catalogue;
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

/** A translator over `catalogue`, bound to `namespace` (empty for the root). */
export function catalogueTranslator(catalogue: Catalogue, namespace = ""): Translate {
  return (key, params) => {
    const full = namespace ? `${namespace}.${key}` : key;
    const message = lookup(catalogue, full);
    return message === null ? full : formatMessage(message, params);
  };
}

/** A translator over this package's catalogue, bound to `namespace`. */
export function translator(namespace: string): Translate {
  return catalogueTranslator(en, namespace);
}

/** The viewers' own strings (the catalogue root). */
export const viewerT: Translate = catalogueTranslator(en);
