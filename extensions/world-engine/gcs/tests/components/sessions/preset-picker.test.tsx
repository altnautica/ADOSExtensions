/**
 * The preset picker: the four presets in order, Preview selected by default,
 * and picking a preset then pressing Reconstruct posts exactly that preset to
 * the device's reconstruct route and hands the node + job back; a refusal
 * (session not on the node) surfaces as a toast and starts nothing.
 */

import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";

import { PresetPicker } from "../../../src/components/sessions/PresetPicker";
import { ToastProvider } from "../../../src/components/ui/Toast";
import { CaptureClient } from "../../../src/lib/agent/capture-client";
import messages from "../../../../locales/en.json";
import { callAt, json, stubAgent } from "../../helpers/agent";

afterEach(cleanup);

const SESSION = "visor-1-20261019T101500Z-3fa2";

function renderPicker(reply: Parameters<typeof stubAgent>[0]) {
  const { agent, fetch } = stubAgent(reply);
  const onStarted = vi.fn();
  render(
    <ToastProvider>
      <PresetPicker client={new CaptureClient(agent)} sessionId={SESSION} onStarted={onStarted} />
    </ToastProvider>,
  );
  return { fetch, onStarted };
}

describe("PresetPicker", () => {
  it("offers Preview / HQ / Tour / Warehouse with Preview selected", () => {
    renderPicker(() => json(200, {}));
    const radios = screen.getAllByRole("radio") as HTMLInputElement[];
    expect(radios.map((r) => r.value)).toEqual(["preview", "hq", "tour", "warehouse"]);
    expect(radios.find((r) => r.checked)?.value).toBe("preview");
  });

  it("posts the picked preset to the session's reconstruct route", async () => {
    const { fetch, onStarted } = renderPicker(() =>
      json(200, { ok: true, node: "workstation-1", job_id: "j-42" }),
    );
    fireEvent.click(screen.getByText(messages.sessions.preset.tour));
    fireEvent.click(screen.getByRole("button", { name: messages.sessions.reconstruct }));
    await waitFor(() => expect(onStarted).toHaveBeenCalled());

    const { path, init } = callAt(fetch);
    expect(path).toBe(`atlas/sessions/${SESSION}/reconstruct`);
    expect(init.method).toBe("POST");
    expect(JSON.parse(String(init.body))).toEqual({ preset: "tour" });
    expect(onStarted).toHaveBeenCalledWith({ node: "workstation-1", jobId: "j-42", preset: "tour" });
  });

  it("reports a refusal and starts nothing", async () => {
    const { onStarted } = renderPicker(() => json(409, { error: "E_NOT_UPLOADED" }));
    fireEvent.click(screen.getByRole("button", { name: messages.sessions.reconstruct }));
    await screen.findByText(messages.sessions.errNotUploaded);
    expect(onStarted).not.toHaveBeenCalled();
  });
});
