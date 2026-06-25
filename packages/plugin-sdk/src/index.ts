/**
 * @altnautica/plugin-sdk. TypeScript SDK for ADOS Mission Control
 * plugins. Provides the wire protocol shape, the postMessage transport,
 * the high-level `PluginContext`, and the `definePlugin` lifecycle
 * helper. Test harnesses live at `@altnautica/plugin-sdk/harness`.
 */

export {
  HostError,
  PROTOCOL_VERSION,
  TELEMETRY_TOPICS,
  type RpcEnvelope,
  type RpcError,
  type EnvelopeType,
  type TelemetryTopic,
} from "./protocol";

export {
  createWindowTransport,
  MemoryTransport,
  type Transport,
} from "./transport";

export { PluginClient } from "./client";

export {
  createPluginContext,
  type CreateContextOptions,
  type PluginContext,
  type NotificationPayload,
  type RecordingMark,
  type MissionUpdate,
} from "./api";

export {
  definePlugin,
  type DefinePluginOptions,
  type PluginInfo,
  type PluginInstance,
  type PluginLifecycle,
} from "./definePlugin";

export {
  contributesModel,
  type PluginContributes,
  type PluginParameterContribution,
  type ParameterSchema,
  type ParameterSchemaType,
  type ParameterWidget,
  type ParameterBinding,
  type ParameterUi,
  type PluginTabContribution,
  type NodeProfile,
  type PluginSettingsContribution,
  type PluginModelContribution,
  type PluginModelBoardVariant,
  type PluginEntrypointContribution,
  type PluginMissionTemplateContribution,
  type PluginMapOverlayContribution,
  type PluginSkillContribution,
  type SkillCategory,
  type SkillArmRequirement,
  type PluginSlotContribution,
  type PluginSlotName,
} from "./manifest";

export {
  validateValue,
  clampValue,
  inferWidget,
  resolveBinding,
  defaultFor,
  PARAMETER_WIDGETS,
  PARAMETER_BINDINGS,
  type ValidationResult,
} from "./parameters";
