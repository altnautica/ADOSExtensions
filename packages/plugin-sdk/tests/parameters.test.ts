import { describe, it, expect } from "vitest";

import {
  clampValue,
  contributesModel,
  defaultFor,
  inferWidget,
  resolveBinding,
  validateValue,
  type ParameterSchema,
  type PluginContributes,
  type PluginParameterContribution,
} from "../src/index";

describe("validateValue", () => {
  it("accepts an in-range number and rejects out of range", () => {
    const schema: ParameterSchema = { type: "number", minimum: 3, maximum: 30 };
    expect(validateValue(schema, 8).ok).toBe(true);
    expect(validateValue(schema, 2).ok).toBe(false);
    expect(validateValue(schema, 31).ok).toBe(false);
    expect(validateValue(schema, "8" as unknown).ok).toBe(false);
  });

  it("enforces integer-ness", () => {
    const schema: ParameterSchema = { type: "integer" };
    expect(validateValue(schema, 4).ok).toBe(true);
    expect(validateValue(schema, 4.5).ok).toBe(false);
  });

  it("checks enum membership before type", () => {
    const schema: ParameterSchema = {
      type: "string",
      enum: ["uvc-0", "uvc-1", "csi-0"],
    };
    expect(validateValue(schema, "uvc-1").ok).toBe(true);
    expect(validateValue(schema, "nope").ok).toBe(false);
  });

  it("applies a string pattern", () => {
    const schema: ParameterSchema = { type: "string", pattern: "^cam-[0-9]+$" };
    expect(validateValue(schema, "cam-3").ok).toBe(true);
    expect(validateValue(schema, "cam").ok).toBe(false);
  });
});

describe("clampValue", () => {
  it("clamps to the inclusive bounds", () => {
    const schema: ParameterSchema = { type: "number", minimum: 3, maximum: 30 };
    expect(clampValue(schema, 1)).toBe(3);
    expect(clampValue(schema, 99)).toBe(30);
    expect(clampValue(schema, 8)).toBe(8);
  });

  it("quantizes to the step and re-clamps", () => {
    const schema: ParameterSchema = {
      type: "number",
      minimum: 0,
      maximum: 10,
      step: 0.5,
    };
    expect(clampValue(schema, 4.3)).toBe(4.5);
    expect(clampValue(schema, 4.1)).toBe(4);
  });

  it("rounds integers", () => {
    const schema: ParameterSchema = { type: "integer", minimum: 1, maximum: 8 };
    expect(clampValue(schema, 4.6)).toBe(5);
  });
});

describe("inferWidget", () => {
  it("uses an explicit widget when valid", () => {
    expect(inferWidget({ type: "number" }, { widget: "range" })).toBe("range");
    expect(inferWidget({ type: "string" }, { widget: "model" })).toBe("model");
  });

  it("infers enum, boolean, number, string from the schema", () => {
    expect(inferWidget({ type: "string", enum: ["a", "b"] })).toBe("enum");
    expect(inferWidget({ type: "boolean" })).toBe("boolean");
    expect(inferWidget({ type: "integer" })).toBe("number");
    expect(inferWidget({ type: "string" })).toBe("string");
  });
});

describe("resolveBinding", () => {
  it("defaults to plugin.config and honors known bindings", () => {
    const base: PluginParameterContribution = {
      key: "k",
      schema: { type: "number" },
    };
    expect(resolveBinding(base)).toBe("plugin.config");
    expect(resolveBinding({ ...base, binding: "engine.detector" })).toBe(
      "engine.detector",
    );
    expect(
      resolveBinding({
        ...base,
        binding: "bogus" as unknown as PluginParameterContribution["binding"],
      }),
    ).toBe("plugin.config");
  });
});

describe("defaultFor", () => {
  it("prefers the declared default, then enum head, then a type empty", () => {
    expect(defaultFor({ type: "number", default: 8 })).toBe(8);
    expect(defaultFor({ type: "string", enum: ["uvc-0", "uvc-1"] })).toBe(
      "uvc-0",
    );
    expect(defaultFor({ type: "boolean" })).toBe(false);
    expect(defaultFor({ type: "number", minimum: 2 })).toBe(2);
    expect(defaultFor({ type: "string" })).toBe("");
  });
});

describe("contributesModel", () => {
  it("returns the model unchanged (identity helper for typing)", () => {
    const model: PluginContributes = {
      tabs: [{ id: "settings", profile: ["drone"], title: "Settings" }],
      parameters: [
        {
          key: "follow_distance_m",
          schema: { type: "number", minimum: 3, maximum: 30, default: 8 },
          ui: { widget: "range", label: "settings.followDistance" },
        },
        {
          key: "detector",
          schema: { type: "string" },
          binding: "engine.detector",
          ui: { widget: "model", task: "detection" },
        },
      ],
      models: [
        {
          id: "coco-person",
          task: "detection",
          board_variants: [
            { board_match: "rk3588", runtime: "rknn", min_tops: 6 },
          ],
        },
      ],
    };
    expect(contributesModel(model)).toBe(model);
    // The model picker parameter is bound to the shared detector.
    expect(resolveBinding(model.parameters![1]!)).toBe("engine.detector");
  });
});
