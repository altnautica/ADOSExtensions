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

/**
 * Which compute the perception (vision) pipeline is running on for the
 * selected node. `local` = the node's own accelerator; `offload` = a
 * paired compute/workstation node runs it; `hybrid` = a mix; `none` = no
 * pipeline active. `null` when the host has not resolved a tier yet.
 */
export type PerceptionTier = "local" | "offload" | "hybrid" | "none";

/**
 * Read-only snapshot of where the perception pipeline runs. The shape
 * matches the host's `perception.read` response. `offloadTarget` names
 * the compute node serving detections when `tier` is `offload`/`hybrid`,
 * otherwise `null`. `npuTops` / `hasAccelerator` describe the selected
 * node's own inference hardware and are absent when the host does not
 * report them.
 */
export interface PerceptionTierInfo {
  tier: PerceptionTier | null;
  offloadTarget: string | null;
  npuTops?: number;
  hasAccelerator?: boolean;
}

/** Discrete identity-lock state of a track this frame. `locked` = the
 * tracker held the identity; `uncertain` = a provisional association;
 * `lost` = the track could not be re-associated. */
export type PerceptionLockState = "locked" | "uncertain" | "lost";

/** Pixel-space bounding box (origin top-left) expressed in the source
 * frame's own resolution. Scale by (renderedWidth / frameWidth) to draw
 * it over a rendered video element. */
export interface PerceptionBoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

/**
 * One detection the host streams to the plugin. This is the serializable
 * subset the host sends over the wire (a strict subset of the GCS's own
 * richer detection model — no live object references or class handles).
 */
export interface PerceptionDetection {
  /** Bounding box in source-frame pixels. */
  bbox: PerceptionBoundingBox;
  /** Human-readable class label (e.g. `person`, `car`). */
  classLabel: string;
  /** Detection confidence in the 0..1 range. */
  confidence: number;
  /** Stable track id across frames. Present only for tracking models;
   * `null`/absent when the source does not track. */
  trackId?: number | null;
  /** Discrete identity-lock state this frame. `null`/absent when the
   * source does not report a lock state. */
  lockState?: PerceptionLockState | null;
}

/**
 * A batch of detections for a single frame from one (model, camera)
 * stream. Carries the source frame size so a plugin can scale boxes to a
 * rendered video element. Matches the host's `perception.detections`
 * event payload.
 */
export interface PerceptionDetectionBatch {
  /** Id of the model that produced the detections. */
  modelId: string;
  /** Id of the camera the frame came from. */
  cameraId: string;
  /** Monotonic frame counter for the stream. */
  frameId: number;
  /** Capture timestamp of the frame, epoch milliseconds. */
  tsMs: number;
  /** Resolution the detection coordinates are expressed in. */
  frameWidth: number;
  frameHeight: number;
  /** The detections found in this frame. Empty when nothing was seen. */
  detections: PerceptionDetection[];
}

/**
 * Read-only health of the perception streaming session. Matches the
 * host's `perception.health` response. `session` is the transport state
 * (`opening` before the first batch, `live` while flowing, `stalled`
 * when batches stop arriving, `closed` after teardown). `feed` reflects
 * detection freshness independent of the transport. `ageMs` is the age
 * of the newest batch, `batchesPerSecond` the measured throughput, and
 * `boundNode` the compute node currently serving the session — each
 * `null` when the host has no reading yet.
 */
export interface PerceptionSessionHealth {
  session: "opening" | "live" | "stalled" | "closed";
  feed: "idle" | "fresh" | "stale";
  ageMs: number | null;
  batchesPerSecond: number | null;
  boundNode: string | null;
}

export interface PluginContext {
  client: PluginClient;
  telemetry: {
    subscribe<TArgs = unknown>(
      topic: string,
      handler: (args: TArgs) => void,
    ): Promise<() => void>;
  };
  /**
   * Read-only view of the perception (vision) pipeline. `readTier` and
   * `readSessionHealth` are one-shot reads; `subscribeDetections` opens
   * the host's detection stream (mirrors `telemetry.subscribe`).
   */
  perception: {
    /** Resolve where the perception pipeline runs for the selected node. */
    readTier(): Promise<PerceptionTierInfo>;
    /**
     * Subscribe to the host-pushed detection stream. Resolves with an
     * unsubscribe function that stops the stream both locally and on the
     * host.
     */
    subscribeDetections(
      handler: (batch: PerceptionDetectionBatch) => void,
    ): Promise<() => void>;
    /** Read the current perception streaming-session health snapshot. */
    readSessionHealth(): Promise<PerceptionSessionHealth>;
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
  events: {
    /**
     * Subscribe to a one-way host-pushed event topic. The host streams
     * these on the same channel as theme/host-prop pushes (no request /
     * response round-trip): video-overlay host props, an agent plugin's
     * state read-back, and any other topic the host forwards to the
     * iframe. Returns an unsubscribe function.
     */
    subscribe<T = unknown>(
      topic: string,
      handler: (args: T) => void,
    ): () => void;
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
    perception: {
      readTier: () =>
        client.request<PerceptionTierInfo>(
          "perception.read",
          "perception.read",
          {},
        ),
      subscribeDetections: (handler) =>
        client.subscribePerception<PerceptionDetectionBatch>(handler),
      readSessionHealth: () =>
        client.request<PerceptionSessionHealth>(
          "perception.health",
          "perception.read",
          {},
        ),
    },
    command: {
      send: (command, args) =>
        client.request("command.send", "command.send", { command, args }),
    },
    notifications: {
      publish: (payload) =>
        client.request("notification.publish", "ui.slot.notification-channel", payload),
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
    events: {
      subscribe: (topic, handler) => client.on(topic, handler),
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
