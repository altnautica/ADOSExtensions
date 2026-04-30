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

/**
 * The single entry point a plugin module exposes. Plugin authors call
 * `definePlugin({...})`. The host loads the bundle inside its sandboxed
 * iframe; the SDK arranges the mount call once the document is ready.
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

  void Promise.resolve(lifecycle.mount(ctx, info)).then(() => {
    mounted = true;
  });

  return {
    info,
    ctx,
    async unmount() {
      if (!mounted) return;
      mounted = false;
      if (lifecycle.unmount) {
        await Promise.resolve(lifecycle.unmount(ctx, info));
      }
      ctx.client.dispose();
    },
  };
}
