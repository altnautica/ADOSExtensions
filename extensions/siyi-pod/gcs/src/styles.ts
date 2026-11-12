/**
 * Panel stylesheet injection.
 *
 * The host loads only the bundle into the sandboxed frame, so `plugin.css`
 * is bundled as text and added as a `<style>` element at mount.
 */

export function injectStyles(cssText: string): void {
  if (document.head.querySelector("style[data-siyi-pod]")) return;
  const style = document.createElement("style");
  style.setAttribute("data-siyi-pod", "");
  style.textContent = cssText;
  document.head.appendChild(style);
}
