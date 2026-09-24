import { describe, expect, it } from "vitest";

import { formatMessage, translator } from "../../src/host/i18n";
import messages from "../../../locales/en.json";

describe("formatMessage", () => {
  it("fills every named placeholder, numbers included", () => {
    expect(formatMessage("{a} of {b} from {a}", { a: "3", b: 7 })).toBe("3 of 7 from 3");
  });

  it("leaves a placeholder with no value visible rather than blank", () => {
    expect(formatMessage("Reconstructed with {backend}")).toBe("Reconstructed with {backend}");
  });

  it("picks the plural arm and substitutes # with the number", () => {
    const msg = "{n, plural, one {# keyframe} other {# keyframes}} ingested";
    expect(formatMessage(msg, { n: 1 })).toBe("1 keyframe ingested");
    expect(formatMessage(msg, { n: 0 })).toBe("0 keyframes ingested");
    expect(formatMessage(msg, { n: 12 })).toBe("12 keyframes ingested");
  });

  it("prefers an exact =N arm over the category arm", () => {
    const msg = "{n, plural, =0 {no jobs} one {# job} other {# jobs}}";
    expect(formatMessage(msg, { n: 0 })).toBe("no jobs");
    expect(formatMessage(msg, { n: 1 })).toBe("1 job");
    expect(formatMessage(msg, { n: 5 })).toBe("5 jobs");
  });

  it("fills a placeholder inside a plural arm", () => {
    const msg = "{n, plural, one {# frame from {drone}} other {# frames from {drone}}}";
    expect(formatMessage(msg, { n: 2, drone: "drone-1" })).toBe("2 frames from drone-1");
    expect(formatMessage(msg, { n: 1, drone: "drone-1" })).toBe("1 frame from drone-1");
  });

  it("resolves the plural and the plain placeholders of one message", () => {
    expect(
      formatMessage("{count, plural, one {# job} other {# jobs}} on {node}", {
        count: 2,
        node: "ws-1",
      }),
    ).toBe("2 jobs on ws-1");
  });

  it("leaves a plural untouched when its value is not a number", () => {
    const msg = "{n, plural, one {# frame} other {# frames}}";
    expect(formatMessage(msg, { n: "many" })).toBe(msg);
    expect(formatMessage(msg)).toBe(msg);
  });
});

describe("translator", () => {
  it("reads a namespaced key from the catalogue and interpolates it", () => {
    const t = translator("atlas");
    expect(t("reconstructedWith", { backend: "brush" })).toBe(
      messages.atlas.reconstructedWith.replace("{backend}", "brush"),
    );
  });

  it("renders a missing key as its full dotted path", () => {
    expect(translator("atlas")("noSuchKey")).toBe("atlas.noSuchKey");
    expect(translator("nope.deeper")("x")).toBe("nope.deeper.x");
  });

  it("does not return a branch of the catalogue as a message", () => {
    expect(translator("nodeSettings")("atlas")).toBe("nodeSettings.atlas");
  });
});
