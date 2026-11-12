/**
 * The overlay's stylesheet, inlined into the bundle as text. The host loads
 * only the bundle into the sandboxed frame, so the rules are injected as a
 * `<style>` element at mount (the frame's policy allows inline styles).
 */

import css from "../plugin.css";

const STYLE_ID = "thm-plugin-style";

/** Inject the stylesheet once into `doc`'s head. */
export function injectStyle(doc: Document): void {
  if (doc.getElementById(STYLE_ID)) return;
  const style = doc.createElement("style");
  style.id = STYLE_ID;
  style.textContent = css;
  doc.head.appendChild(style);
}
