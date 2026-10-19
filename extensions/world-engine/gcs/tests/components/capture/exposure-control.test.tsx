/**
 * A lock the cameras did not take is flagged with its reason; an applied
 * lock and auto exposure carry no flag.
 */

import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import { ExposureControl } from "../../../src/components/capture/ExposureControl";
import { ToastProvider } from "../../../src/components/ui/Toast";
import { CaptureClient } from "../../../src/lib/agent/capture-client";
import { coerceExposure } from "../../../src/lib/capture/capture-state";
import messages from "../../../../locales/en.json";
import { json, stubAgent } from "../../helpers/agent";

afterEach(cleanup);

function show(raw: unknown) {
  const { agent } = stubAgent(() => json(200, {}));
  render(
    <ToastProvider>
      <ExposureControl client={new CaptureClient(agent)} exposure={coerceExposure(raw)} onChanged={() => {}} />
    </ToastProvider>,
  );
}

describe("ExposureControl", () => {
  it("flags a requested lock that was not applied, with the reason", () => {
    show({ locked: true, exposure_us: 2000, gain: 2, applied: false, reason: "not_v4l2" });
    expect(
      screen.getByText(messages.capture.exposureNotAppliedReason.replace("{reason}", "not_v4l2")),
    ).toBeTruthy();
  });

  it("flags it without a reason when none is given", () => {
    show({ locked: true, applied: false });
    expect(screen.getByText(messages.capture.exposureNotApplied)).toBeTruthy();
  });

  it("shows no flag for an applied lock or auto exposure", () => {
    show({ locked: true, applied: true });
    expect(screen.queryByRole("status")).toBeNull();
    cleanup();
    show({ locked: false, applied: false, reason: "not_v4l2" });
    expect(screen.queryByRole("status")).toBeNull();
  });
});
