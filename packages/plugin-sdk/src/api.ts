import { PluginClient } from "./client";
import { HostError } from "./protocol";

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

/** One record in the plugin's own cloud namespace. */
export interface PluginRecord {
  collection: string;
  key: string;
  /** Node the record is about, or null for an account-level record. */
  deviceId: string | null;
  data: unknown;
  /** Epoch milliseconds of the last write. */
  updatedAt: number;
  /** Which half of the plugin wrote it last. */
  writtenBy: "gcs" | "agent";
}

/** Which records `PluginRecordsApi.list` returns. */
export interface PluginRecordListOptions {
  /** 1-64 of `[a-z0-9_.-]`. */
  collection: string;
  /** Only records about this node. */
  deviceId?: string;
  /** Page size, 1-100 (default 50). */
  limit?: number;
}

/**
 * The plugin's own cloud records, stored under the signed-in operator's
 * account and namespaced to this plugin by the host. Needs the
 * `cloud.records` grant. A record body is at most 64 KiB of JSON and a plugin
 * holds at most 5000 records. A refusal rejects with a `HostError` whose code
 * is `refused` and whose message is one of `unavailable` (signed out or
 * offline), `invalid_args`, `too_large`, `limit_reached`, `not_permitted`,
 * `failed`.
 */
export interface PluginRecordsApi {
  /** A page of one collection, in key order. */
  list(opts: PluginRecordListOptions): Promise<PluginRecord[]>;
  get(collection: string, key: string): Promise<PluginRecord | null>;
  /** Insert or replace the record at `(collection, key)`. */
  put(
    collection: string,
    key: string,
    data: unknown,
    opts?: { deviceId?: string },
  ): Promise<void>;
  /** Delete one record; deleting a missing key succeeds. */
  remove(collection: string, key: string): Promise<void>;
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
    /**
     * Send a command. The host asks the operator to approve it first, so the
     * promise waits for the operator's decision instead of the default
     * request deadline; the host always answers, denying a prompt the
     * operator leaves unanswered. A denial or a command the host does not
     * allow rejects with a `HostError` whose code is `refused`.
     */
    send(command: string, args?: unknown): Promise<unknown>;
  };
  notifications: {
    /** Rejects with code `refused` when the host drops it (rate limit). */
    publish(payload: NotificationPayload): Promise<unknown>;
  };
  recording: {
    /** Rejects with code `refused` when nothing is recording. */
    mark(payload: RecordingMark): Promise<unknown>;
  };
  mission: {
    read(missionId: string): Promise<unknown>;
    /**
     * Replace the mission. Operator-approved like `command.send`, so it waits
     * for the operator's decision and the upload instead of the default
     * request deadline, and rejects with code `refused` when it is denied.
     */
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
    /**
     * Open delivery of one GCS plugin-bus topic (another plugin's published
     * events). Needs the `event.subscribe`
     * grant. Resolves with a function that stops delivery on the host too.
     */
    listen<T = unknown>(
      topic: string,
      handler: (args: T) => void,
    ): Promise<() => void>;
    /** Publish an event on the GCS plugin bus. Needs `event.publish`. */
    publish(topic: string, payload: unknown): Promise<unknown>;
  };
  /** The plugin's own cloud records. Needs `cloud.records`. */
  records: PluginRecordsApi;
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

/**
 * Request options for methods the host holds for operator approval. A human
 * decision can take longer than any fixed deadline, and a client timeout would
 * report failure for an action the operator then approves. The host bounds the
 * wait itself: an unanswered prompt is denied.
 */
const OPERATOR_CONFIRMED = { timeoutMs: Number.POSITIVE_INFINITY };

/**
 * The host answers a refused action (operator denial, a command outside its
 * allowlist, a rate limit, nothing recording) with the result
 * `{ ok: false, error }` rather than an RPC error. Reject with that reason so a
 * caller never reads a refusal as done; any other result passes through.
 */
async function unlessRefused<T>(pending: Promise<T>): Promise<T> {
  const result = await pending;
  if (
    typeof result === "object" &&
    result !== null &&
    "ok" in result &&
    result.ok === false
  ) {
    const reason = "error" in result ? result.error : undefined;
    throw new HostError(
      "refused",
      typeof reason === "string" && reason.length > 0
        ? reason
        : "the host refused the request",
    );
  }
  return result;
}

/** Unwrap a `{ ok: true, result }` answer, rejecting a refusal. */
async function hostResult<T>(pending: Promise<unknown>): Promise<T> {
  const answer = await unlessRefused(pending);
  if (typeof answer !== "object" || answer === null || !("result" in answer)) {
    throw new HostError("refused", "the host answered without a result");
  }
  // The host owns the result shape for each method; the SDK types it per call.
  const result = answer.result as T;
  return result;
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
        unlessRefused(
          client.request(
            "command.send",
            "command.send",
            { command, args },
            OPERATOR_CONFIRMED,
          ),
        ),
    },
    notifications: {
      publish: (payload) =>
        unlessRefused(
          client.request(
            "notification.publish",
            "ui.slot.notification-channel",
            payload,
          ),
        ),
    },
    recording: {
      mark: (payload) =>
        unlessRefused(
          client.request("recording.mark", "recording.write", payload),
        ),
    },
    mission: {
      read: (missionId) =>
        client.request("mission.read", "mission.read", { missionId }),
      write: (update) =>
        unlessRefused(
          client.request(
            "mission.write",
            "mission.write",
            update,
            OPERATOR_CONFIRMED,
          ),
        ),
    },
    config: {
      onChange: (handler) => client.on("config.changed", handler),
    },
    events: {
      subscribe: (topic, handler) => client.on(topic, handler),
      listen: async (topic, handler) => {
        // The host delivers bus events with the topic as the method.
        const off = client.on(topic, handler);
        try {
          await client.request("events.subscribe", "event.subscribe", { topic });
        } catch (err) {
          off();
          throw err;
        }
        return () => {
          off();
          void client
            .request("events.unsubscribe", "", { topic })
            .catch(() => {});
        };
      },
      publish: (topic, payload) =>
        client.request("events.publish", "event.publish", { topic, payload }),
    },
    records: {
      list: (opts) =>
        hostResult<PluginRecord[]>(client.request("records.list", "cloud.records", opts)),
      get: (collection, key) =>
        hostResult<PluginRecord | null>(
          client.request("records.get", "cloud.records", { collection, key }),
        ),
      put: async (collection, key, data, opts) => {
        await unlessRefused(
          client.request("records.put", "cloud.records", {
            collection,
            key,
            data,
            ...(opts?.deviceId !== undefined ? { deviceId: opts.deviceId } : {}),
          }),
        );
      },
      remove: async (collection, key) => {
        await unlessRefused(
          client.request("records.remove", "cloud.records", { collection, key }),
        );
      },
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
