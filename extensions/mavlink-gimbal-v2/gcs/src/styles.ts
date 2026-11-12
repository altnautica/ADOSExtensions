/**
 * Panel stylesheet injection.
 *
 * The host loads only the bundle into the sandboxed frame, so `plugin.css`
 * is bundled as text and added as a `<style>` element at mount.
 */

const STYLE_ID = "agm-styles";

export function injectStyles(cssText: string): void {
  if (document.getElementById(STYLE_ID)) return;
  const style = document.createElement("style");
  style.id = STYLE_ID;
  style.textContent = cssText;
  document.head.appendChild(style);
}
