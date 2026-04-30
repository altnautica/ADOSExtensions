import { PluginClient } from "./client";

/**
 * High-level shorthand wrappers grouped by domain so plugin code reads
 * naturally:
 *
 *   const ctx = createPluginContext();
 *   await ctx.telemetry.subscribe("battery", (s) => store.ingest(s));
 *   await ctx.notifications.publish({ ... });
 *
 * The wrappers all delegate to a single PluginClient. Plugins that
 * need finer control can drop down to `ctx.client.request(...)`.
 */

export interface NotificationPayload {
  channelId: string;
  severity: "info" | "warning" | "critical";
  title: string;
  body?: string;
  meta?: Record<string, unknown>;
}

export interface RecordingMark {
  label: string;
  meta?: Record<string, unknown>;
}

export interface MissionUpdate {
  /** Opaque mission id assigned by the host. */
  missionId: string;
  /** Host-relative path or marker id depending on action. */
  path?: string;
  /** Free-form payload validated by the host. */
  payload?: unknown;
}

export interface PluginContext {
  client: PluginClient;
  telemetry: {
    subscribe<TArgs = unknown>(
      topic: string,
      handler: (args: TArgs) => void,
    ): Promise<() => void>;
  };
  command: {
    send(command: string, args?: unknown): Promise<unknown>;
  };
  notifications: {
    publish(payload: NotificationPayload): Promise<unknown>;
  };
  recording: {
    mark(payload: RecordingMark): Promise<unknown>;
  };
  mission: {
    read(missionId: string): Promise<unknown>;
    write(update: MissionUpdate): Promise<unknown>;
  };
  config: {
    onChange<T = unknown>(handler: (next: T) => void): () => void;
  };
  theme: {
    onChange(
      handler: (vars: Record<string, string>) => void,
    ): () => void;
  };
  i18n: {
    /**
     * Resolve a key against the locale bundle the host streams in. Falls
     * back to the key itself when no bundle is registered yet.
     */
    t(key: string, params?: Record<string, string | number>): string;
  };
}

export interface CreateContextOptions {
  client?: PluginClient;
  /** Initial locale bundle. Plugins ship their own JSON in /locales. */
  locale?: Record<string, string>;
}

export function createPluginContext(
  opts: CreateContextOptions = {},
): PluginContext {
  const client = opts.client ?? new PluginClient();
  const localeBundle: Record<string, string> = opts.locale ?? {};

  return {
    client,
    telemetry: {
      subscribe: (topic, handler) => client.subscribeTelemetry(topic, handler),
    },
    command: {
      send: (command, args) =>
        client.request("command.send", "command.send", { command, args }),
    },
    notifications: {
      publish: (payload) =>
        client.request("notification.publish", "ui.slot.notification", payload),
    },
    recording: {
      mark: (payload) =>
        client.request("recording.mark", "recording.write", payload),
    },
    mission: {
      read: (missionId) =>
        client.request("mission.read", "mission.read", { missionId }),
      write: (update) =>
        client.request("mission.write", "mission.write", update),
    },
    config: {
      onChange: (handler) => client.on("config.changed", handler),
    },
    theme: {
      onChange: (handler) =>
        client.on<Record<string, string>>("theme.changed", handler),
    },
    i18n: {
      t: (key, params) => formatLocale(localeBundle, key, params),
    },
  };
}

function formatLocale(
  bundle: Record<string, string>,
  key: string,
  params?: Record<string, string | number>,
): string {
  const tpl = bundle[key];
  if (tpl === undefined) return key;
  if (!params) return tpl;
  return tpl.replace(/\{(\w+)\}/g, (_, name) => {
    const value = params[name];
    return value === undefined ? `{${name}}` : String(value);
  });
}
