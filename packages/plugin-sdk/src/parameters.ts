/**
 * Pure helpers for the declarative parameter model. These mirror the host's
 * validator (`ADOSMissionControl/src/lib/plugins/parameters/schema.ts`) so a
 * plugin author can unit-test the values their settings produce against the
 * exact rules the GCS applies on commit — validate, clamp/quantize, infer the
 * default, and resolve the effective binding/widget.
 *
 * Dependency-free on purpose: the subset is small (number/integer/boolean/
 * string/enum with min/max/step/pattern/default), and the SDK carries no
 * JSON-Schema runtime.
 */

import type {
  ParameterBinding,
  ParameterSchema,
  ParameterUi,
  ParameterWidget,
  PluginParameterContribution,
} from "./manifest";

export interface ValidationResult {
  ok: boolean;
  /** Human-readable reason when `ok` is false. */
  error?: string;
}

const WIDGETS: ReadonlySet<string> = new Set([
  "number",
  "range",
  "boolean",
  "enum",
  "string",
  "model",
  "model_upload",
  "group",
]);

const BINDINGS: ReadonlySet<string> = new Set([
  "plugin.config",
  "engine.detector",
  "agent.config",
]);

function isFiniteNumber(v: unknown): v is number {
  return typeof v === "number" && Number.isFinite(v);
}

/**
 * Validate a committed value against a schema. enum membership short-circuits
 * the type checks (an enum may legitimately mix scalar types).
 */
export function validateValue(
  schema: ParameterSchema,
  value: unknown,
): ValidationResult {
  if (schema.enum) {
    const present = schema.enum.some((e) => e === value);
    return present
      ? { ok: true }
      : { ok: false, error: "is not one of the allowed values" };
  }
  switch (schema.type) {
    case "number":
    case "integer": {
      if (!isFiniteNumber(value)) {
        return { ok: false, error: "must be a number" };
      }
      if (schema.type === "integer" && !Number.isInteger(value)) {
        return { ok: false, error: "must be an integer" };
      }
      if (isFiniteNumber(schema.minimum) && value < schema.minimum) {
        return { ok: false, error: `must be >= ${schema.minimum}` };
      }
      if (isFiniteNumber(schema.maximum) && value > schema.maximum) {
        return { ok: false, error: `must be <= ${schema.maximum}` };
      }
      return { ok: true };
    }
    case "boolean":
      return typeof value === "boolean"
        ? { ok: true }
        : { ok: false, error: "must be a boolean" };
    case "string": {
      if (typeof value !== "string") {
        return { ok: false, error: "must be a string" };
      }
      if (schema.pattern) {
        let re: RegExp | null = null;
        try {
          re = new RegExp(schema.pattern);
        } catch {
          re = null;
        }
        if (re && !re.test(value)) {
          return { ok: false, error: "does not match the required pattern" };
        }
      }
      return { ok: true };
    }
    default:
      return { ok: true };
  }
}

/**
 * Coerce + clamp a value toward schema-validity: numbers clamp to
 * `[minimum, maximum]` and quantize to `step`; everything else is returned
 * unchanged. Returns the value to persist.
 */
export function clampValue(
  schema: ParameterSchema,
  value: unknown,
): string | number | boolean | undefined {
  if (schema.type === "number" || schema.type === "integer") {
    if (!isFiniteNumber(value)) return value as undefined;
    let v = value;
    if (isFiniteNumber(schema.minimum)) v = Math.max(v, schema.minimum);
    if (isFiniteNumber(schema.maximum)) v = Math.min(v, schema.maximum);
    if (isFiniteNumber(schema.step) && schema.step > 0) {
      const base = isFiniteNumber(schema.minimum) ? schema.minimum : 0;
      v = base + Math.round((v - base) / schema.step) * schema.step;
      if (isFiniteNumber(schema.minimum)) v = Math.max(v, schema.minimum);
      if (isFiniteNumber(schema.maximum)) v = Math.min(v, schema.maximum);
    }
    if (schema.type === "integer") v = Math.round(v);
    return v;
  }
  if (typeof value === "string" || typeof value === "boolean") return value;
  return undefined;
}

/** Infer the rendered control from the schema when `ui.widget` is omitted. */
export function inferWidget(
  schema: ParameterSchema,
  ui?: ParameterUi,
): ParameterWidget {
  if (ui?.widget && WIDGETS.has(ui.widget)) return ui.widget;
  if (schema.enum && schema.enum.length > 0) return "enum";
  switch (schema.type) {
    case "boolean":
      return "boolean";
    case "number":
    case "integer":
      return "number";
    case "string":
    default:
      return "string";
  }
}

/** Resolve a parameter's effective binding, defaulting to `plugin.config`. */
export function resolveBinding(
  param: PluginParameterContribution,
): ParameterBinding {
  return param.binding && BINDINGS.has(param.binding)
    ? param.binding
    : "plugin.config";
}

/** The schema's declared default, or a type-appropriate empty default. */
export function defaultFor(
  schema: ParameterSchema,
): string | number | boolean {
  if (schema.default !== undefined) return schema.default;
  if (schema.enum && schema.enum.length > 0) return schema.enum[0]!;
  switch (schema.type) {
    case "boolean":
      return false;
    case "number":
    case "integer":
      return isFiniteNumber(schema.minimum) ? schema.minimum : 0;
    case "string":
    default:
      return "";
  }
}

export const PARAMETER_WIDGETS = WIDGETS;
export const PARAMETER_BINDINGS = BINDINGS;
