/**
 * Reconstruct's own strings (`locales/en.json`), looked up by dotted key
 * under a namespace; the message format is the shared one
 * (`@altnautica/mapping-ui/host/i18n`).
 */

import { catalogueTranslator, type Translate } from "@altnautica/mapping-ui/host/i18n";

import en from "../../../locales/en.json";

/** A translator bound to one namespace (`"node"`, `"reconstruct"`). */
export function translator(namespace: string): Translate {
  return catalogueTranslator(en, namespace);
}
