/**
 * Manifest `contributes` model types + helpers.
 *
 * A plugin's `manifest.yaml` declares what it adds to the GCS through a
 * `gcs.contributes` block. The host parses that block into validated
 * contribution rows; this module mirrors the same shapes so plugin authors
 * get a typed, documented surface to build their manifest against without
 * depending on the GCS source tree. The authoritative parsers live in
 * `ADOSMissionControl/src/lib/plugins/{parameters,contributions}/parse.ts` and
 * `.../parameters/schema.ts`; these declarations track them field-for-field.
 *
 * The two iframe-mounting kinds the host already supported — skills and the
 * panel/overlay/notification slots — are declared here too so the full
 * `contributes` surface lives in one place.
 *
 * Everything is plain serializable data: a manifest is YAML, so these types
 * describe the parsed-YAML object, not a runtime API. `contributesModel()` is
 * a tiny identity helper that gives an author editor autocomplete + a
 * compile-time check while authoring a manifest fixture or codegen.
 */

/* ------------------------------------------------------------------ *
 * Parameters (gcs.contributes.parameters[])
 * ------------------------------------------------------------------ */

/** The JSON-Schema (Draft-07 subset) scalar types a parameter value takes. */
export type ParameterSchemaType =
  | "number"
  | "integer"
  | "boolean"
  | "string";

/**
 * A JSON-Schema Draft-07 subset describing one parameter's value. Only the
 * keywords a drone-plugin parameter needs are honored by the host; unknown
 * keywords are ignored (forward-compatible), never an error.
 */
export interface ParameterSchema {
  type: ParameterSchemaType;
  /** Numeric inclusive bounds (number/integer only). */
  minimum?: number;
  maximum?: number;
  /** UI/quantization step (number/integer only). Advisory for clamping. */
  step?: number;
  /** Allowed values. When present the value must be one of these; pairs with
   * the `enum` widget. */
  enum?: ReadonlyArray<string | number | boolean>;
  /** Regex source applied to string values. */
  pattern?: string;
  /** Default applied when the stored value is absent. */
  default?: string | number | boolean;
}

/**
 * The control a parameter renders as. Inferred from `schema.type` when the
 * manifest omits it; `range`/`model`/`model_upload`/`group` are always
 * explicit. A `model` widget renders the vision-model picker; pair it with
 * `binding: "engine.detector"` and `ui.task` to filter the catalog.
 */
export type ParameterWidget =
  | "number"
  | "range"
  | "boolean"
  | "enum"
  | "string"
  | "model"
  | "model_upload"
  | "group";

/**
 * Where a parameter's committed value is written. `plugin.config` (default)
 * writes the per-node plugin config store the agent half reads live;
 * `engine.detector` writes the shared vision detector selection;
 * `agent.config` writes a whitelisted system key.
 */
export type ParameterBinding =
  | "plugin.config"
  | "engine.detector"
  | "agent.config";

/** The presentation layer for a parameter (the uiSchema split): never affects
 * validation, only how the value is shown and grouped. */
export interface ParameterUi {
  widget?: ParameterWidget;
  /** i18n key or literal label. */
  label?: string;
  /** Section grouping in the rendered panel. */
  group?: string;
  /** Helper/description text. */
  help?: string;
  /** Conditional reveal: show this control only when another parameter's
   * committed value equals `equals`. */
  visible_if?: { key: string; equals: string | number | boolean };
  /** Model-widget filter — the vision task a `model`/`model_upload` picker
   * lists, e.g. "detection" | "reid" | "depth". */
  task?: string;
  /** Sort hint within a group. */
  order?: number;
}

/**
 * One `gcs.contributes.parameters[]` entry: a declarative control the GCS
 * renders natively (no iframe) in the plugin's settings, with the committed
 * value routed to `binding`.
 */
export interface PluginParameterContribution {
  /** Config key written on commit (unique within the plugin). */
  key: string;
  schema: ParameterSchema;
  ui?: ParameterUi;
  /** Defaults to `plugin.config` when absent. */
  binding?: ParameterBinding;
}

/* ------------------------------------------------------------------ *
 * Node-detail tabs (gcs.contributes.tabs[])
 * ------------------------------------------------------------------ */

/** The node profiles a tab can be offered on. */
export type NodeProfile = "drone" | "ground-station" | "compute";

/**
 * One `gcs.contributes.tabs[]` entry: a detail tab the plugin mounts on a
 * node's detail panel. The slot is always `node.detail.tab`
 * (capability `ui.slot.node-detail-tab`); an optional `profile` list narrows
 * which node profiles offer the tab.
 */
export interface PluginTabContribution {
  /** Stable id within the plugin. */
  id: string;
  /** Node profiles the tab is offered on. Absent means any profile. */
  profile?: NodeProfile[];
  /** Display title for the tab header (i18n key or literal). */
  title?: string;
  /** lucide-react icon hint. */
  icon?: string;
  /** Sort hint within the tab strip. */
  order?: number;
  /** Bundle entrypoint the host mounts (relative path inside the archive).
   * Omit to use the plugin's default `gcs.entrypoint`. */
  entrypoint?: string;
}

/* ------------------------------------------------------------------ *
 * Settings sections (gcs.contributes.settings[])
 * ------------------------------------------------------------------ */

/**
 * One `gcs.contributes.settings[]` entry: a section in the plugin's settings
 * panel holding native declarative parameters.
 */
export interface PluginSettingsContribution {
  /** Stable id within the plugin. */
  id: string;
  /** Section heading (i18n key or literal). */
  title?: string;
  /** lucide-react icon hint. */
  icon?: string;
  /** Sort hint among sections. */
  order?: number;
  /** Native declarative parameters rendered in this section. */
  parameters?: PluginParameterContribution[];
}

/* ------------------------------------------------------------------ *
 * Models (gcs.contributes.models[])
 * ------------------------------------------------------------------ */

/** One per-board variant under a model contribution. */
export interface PluginModelBoardVariant {
  /** Board fingerprint match (SoC/board substring). */
  board_match?: string;
  /** Inference runtime the variant targets (e.g. an ONNX/RKNN runtime). */
  runtime?: string;
  /** Input resolution hint, e.g. "640x640". */
  input?: string;
  /** Minimum NPU throughput (TOPS) the variant needs. */
  min_tops?: number;
  /** Download source for the model file. */
  source?: string;
  /** Expected SHA-256 of the model file. */
  sha256?: string;
}

/**
 * One `gcs.contributes.models[]` entry: a model the plugin registers for a
 * vision task, with per-board variants the agent selects from. A model
 * contribution makes the model selectable from any `model` parameter whose
 * `ui.task` matches.
 */
export interface PluginModelContribution {
  /** Stable id within the plugin. */
  id: string;
  /** Vision task the model serves, e.g. "detection" | "reid" | "depth". */
  task?: string;
  /** Per-board variants. */
  board_variants?: PluginModelBoardVariant[];
}

/* ------------------------------------------------------------------ *
 * Mission templates + map overlays (simple entrypoint contributions)
 * ------------------------------------------------------------------ */

/** Shared shape for entrypoint-bearing contributions. */
export interface PluginEntrypointContribution {
  /** Stable id within the plugin. */
  id: string;
  /** Display title (i18n key or literal). */
  title?: string;
  /** lucide-react icon hint. */
  icon?: string;
  /** Bundle entrypoint (relative path inside the archive). */
  entrypoint?: string;
}

/** One `gcs.contributes.missionTemplates[]` entry. */
export type PluginMissionTemplateContribution = PluginEntrypointContribution;

/** One `gcs.contributes.mapOverlays[]` entry. */
export type PluginMapOverlayContribution = PluginEntrypointContribution;

/* ------------------------------------------------------------------ *
 * Skills + iframe slots (the two iframe-mounting contribution kinds)
 * ------------------------------------------------------------------ */

/** A skill's category, mapped to the registry's SkillCategory at register
 * time. */
export type SkillCategory = "behavior" | "camera" | "navigation" | "utility";

/** Arm-requirement gate for a skill; absent means "any". */
export type SkillArmRequirement = "any" | "armed" | "disarmed";

/**
 * One `gcs.contributes.skills[]` entry: a flight Skill the plugin adds to the
 * cockpit Skill Bar. v1 honors only the config-write activation + event-read
 * state contract — `activation.via` must be `config` (with a `config_key`) and
 * `state.via` must be `event` (with a `topic`).
 */
export interface PluginSkillContribution {
  /** Unique within the plugin; the registry namespaces it to
   * `${pluginId}:${id}`. */
  id: string;
  /** i18n key or literal label. */
  label?: string;
  /** lucide-react icon name. */
  icon?: string;
  /** Category. */
  category?: SkillCategory;
  /** Whether the skill is an on/off toggle (vs a one-shot). */
  toggle?: boolean;
  /** When true, the host builds a confirm policy before activation. */
  confirm?: boolean;
  /** Arm requirement gate. */
  arm_requirement?: SkillArmRequirement;
  /** Suggested default keyboard/gamepad binding. */
  default_binding?: { key?: string; gamepad_button?: number };
  /** Activation transport — v1 is always a per-node config write. */
  activation: { via: "config"; config_key: string };
  /** State transport — v1 is always an event-topic read. */
  state: { via: "event"; topic: string };
}

/** The UI slots an iframe panel can mount into. */
export type PluginSlotName =
  | "fc.tab"
  | "hardware.tab"
  | "mission.template"
  | "map.overlay"
  | "video.overlay"
  | "notification.channel"
  | "settings.section"
  | "connection.protocol"
  | "recording.processor"
  | "node.detail.tab"
  | "cockpit.panel"
  | "flight.skill";

/**
 * One iframe-slot contribution (a panel/overlay/notification channel). The
 * host mounts a sandboxed iframe for the plugin's bundle into `slot`. Entries
 * under `overlays` default to `video.overlay`; under `notifications` to
 * `notification.channel`.
 */
export interface PluginSlotContribution {
  /** Stable id within the plugin. */
  id: string;
  /** A host-known UI slot the iframe mounts into. Required under `panels`;
   * defaulted under `overlays`/`notifications`. */
  slot?: PluginSlotName;
  /** Display title for the tab/overlay header. */
  title?: string;
  /** lucide-react icon hint. */
  icon?: string;
  /** Sort hint; the host defaults to 60 when absent. */
  order?: number;
}

/* ------------------------------------------------------------------ *
 * The full contributes block
 * ------------------------------------------------------------------ */

/**
 * The `gcs.contributes` block of a plugin manifest. Every field is optional;
 * a plugin contributes only the kinds it needs. Mirrors the union the host
 * parser consumes; both `missionTemplates`/`mission_templates` and
 * `mapOverlays`/`map_overlays` spellings are accepted by the host, the
 * camelCase form is canonical here.
 */
export interface PluginContributes {
  /** Flight skills for the cockpit Skill Bar. */
  skills?: PluginSkillContribution[];
  /** Iframe panels; each declares its own `slot`. */
  panels?: PluginSlotContribution[];
  /** Video overlays (slot defaults to `video.overlay`). */
  overlays?: PluginSlotContribution[];
  /** Notification channels (slot defaults to `notification.channel`). */
  notifications?: PluginSlotContribution[];
  /** Declarative native parameters rendered in the settings panel. */
  parameters?: PluginParameterContribution[];
  /** Node-detail tabs mounted on a node's detail panel. */
  tabs?: PluginTabContribution[];
  /** Settings sections, each holding native parameters. */
  settings?: PluginSettingsContribution[];
  /** Vision-model registrations with per-board variants. */
  models?: PluginModelContribution[];
  /** Planner mission templates. */
  missionTemplates?: PluginMissionTemplateContribution[];
  /** Map surface overlays. */
  mapOverlays?: PluginMapOverlayContribution[];
}

/**
 * Identity helper that types a `contributes` literal while authoring a
 * manifest fixture or codegen — gives editor autocomplete + a compile-time
 * shape check, then returns the value unchanged so it can be serialized to
 * YAML.
 *
 *   const contributes = contributesModel({
 *     tabs: [{ id: "settings", profile: ["drone"], title: "settings.tab" }],
 *     parameters: [
 *       { key: "follow_distance_m", schema: { type: "number", minimum: 3,
 *         maximum: 30, default: 8 }, ui: { widget: "range" } },
 *     ],
 *   });
 */
export function contributesModel(c: PluginContributes): PluginContributes {
  return c;
}
