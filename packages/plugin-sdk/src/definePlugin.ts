import { createPluginContext, type PluginContext } from "./api";

export interface PluginInfo {
  /** Reverse-DNS plugin id from the manifest. */
  id: string;
  version: string;
}

export interface PluginLifecycle {
  mount: (ctx: PluginContext, info: PluginInfo) => Promise<void> | void;
  unmount?: (ctx: PluginContext, info: PluginInfo) => Promise<void> | void;
}

export interface DefinePluginOptions extends PluginLifecycle {
  id: string;
  version: string;
  /** Optional locale bundle bundled at build time. */
  locale?: Record<string, string>;
  /** Optional pre-constructed context (used by tests + scaffolders). */
  context?: PluginContext;
}

export interface PluginInstance {
  info: PluginInfo;
  ctx: PluginContext;
  unmount: () => Promise<void>;
}

/** The message the host posts into the frame before it tears the frame down. */
const HOST_UNMOUNT_MESSAGE = "unmount";

function isHostUnmount(ev: MessageEvent): boolean {
  if (typeof window === "undefined" || ev.source !== window.parent) return false;
  const data: unknown = ev.data;
  return (
    data !== null &&
    typeof data === "object" &&
    "type" in data &&
    data.type === HOST_UNMOUNT_MESSAGE
  );
}

/**
 * The single entry point a plugin module exposes. Plugin authors call
 * `definePlugin({...})`. The host loads the bundle inside its sandboxed
 * iframe; the SDK arranges the mount call once the document is ready.
 *
 * Before the host removes the frame it posts `{ type: "unmount" }` and keeps
 * the document alive for a short grace window (500 ms). The SDK answers that
 * message by running `unmount` (after `mount` settles, if it is still
 * running), so a plugin's teardown hook runs without the plugin listening for
 * the message itself.
 */
export function definePlugin(opts: DefinePluginOptions): PluginInstance {
  const info: PluginInfo = { id: opts.id, version: opts.version };
  const ctx =
    opts.context ?? createPluginContext({ locale: opts.locale });
  const lifecycle: PluginLifecycle = {
    mount: opts.mount,
    unmount: opts.unmount,
  };

  let mounted = false;
  let unmounting: Promise<void> | null = null;

  const mounting = Promise.resolve(lifecycle.mount(ctx, info)).then(() => {
    mounted = true;
  });

  const onWindowMessage = (ev: MessageEvent): void => {
    if (isHostUnmount(ev)) void instance.unmount();
  };
  const hasWindow =
    typeof window !== "undefined" && typeof window.addEventListener === "function";
  if (hasWindow) window.addEventListener("message", onWindowMessage);

  const instance: PluginInstance = {
    info,
    ctx,
    unmount() {
      if (unmounting) return unmounting;
      unmounting = (async () => {
        if (hasWindow) window.removeEventListener("message", onWindowMessage);
        // A mount that rejected leaves nothing to tear down.
        await mounting.catch(() => undefined);
        if (!mounted) return;
        mounted = false;
        if (lifecycle.unmount) {
          await Promise.resolve(lifecycle.unmount(ctx, info));
        }
        ctx.client.dispose();
      })();
      return unmounting;
    },
  };
  return instance;
}
