/**
 * The measured LiDAR–IMU entry step starts from the picked sensor's stored
 * extrinsic and re-initialises when another sensor is picked; saving pushes a
 * new rig version with only that sensor changed.
 */

import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";

import { RigEntryCard } from "../../../src/components/calibration/RigEntryCard";
import { ToastProvider } from "../../../src/components/ui/Toast";
import { CaptureClient } from "../../../src/lib/agent/capture-client";
import { transformFromXyzRpy } from "../../../src/lib/capture/rig-entry";
import type { Rig } from "../../../src/lib/contract/generated";
import messages from "../../../../locales/en.json";
import { callAt, json, stubAgent } from "../../helpers/agent";

afterEach(cleanup);

const RIG: Rig = {
  contract: "ados.capture/1",
  rig_id: "pro-01",
  calib_version: "v1",
  sensors: [
    { id: "lidar_top", kind: "lidar", role: "up", T_body_sensor: transformFromXyzRpy([0.01, 0.02, 0.15], [0, 0, 0]), time_offset_ns: 0 },
    { id: "lidar_back", kind: "lidar", role: "back", T_body_sensor: transformFromXyzRpy([-0.2, 0, 0.05], [0, 0, 180]), time_offset_ns: 0 },
  ],
};

const offsetX = () => Number((screen.getByLabelText(`${messages.calibration.translation} x`) as HTMLInputElement).value);
const yaw = () => Number((screen.getByLabelText(`${messages.calibration.rotation} z`) as HTMLInputElement).value);

describe("RigEntryCard", () => {
  it("re-initialises the fields from the newly picked sensor and saves only it", async () => {
    const { agent, fetch } = stubAgent(() => json(200, { ok: true, rig_id: "pro-01", calib_version: "x" }));
    const onApplied = vi.fn();
    render(
      <ToastProvider>
        <RigEntryCard index={3} target="lidar_imu" rig={RIG} device={new CaptureClient(agent)} onApplied={onApplied} />
      </ToastProvider>,
    );
    expect(offsetX()).toBeCloseTo(0.01, 9);

    // Edit the first sensor, then switch: the edit must not leak across.
    fireEvent.change(screen.getByLabelText(`${messages.calibration.translation} x`), { target: { value: "0.5" } });
    fireEvent.change(screen.getByLabelText(messages.calibration.sensor), { target: { value: "lidar_back" } });
    expect(offsetX()).toBeCloseTo(-0.2, 9);
    expect(Math.abs(yaw())).toBeCloseTo(180, 6);

    fireEvent.click(screen.getByRole("button", { name: messages.calibration.saveToRig }));
    await waitFor(() => expect(onApplied).toHaveBeenCalled());
    const pushed = JSON.parse(String(callAt(fetch).init.body)) as Rig;
    expect(callAt(fetch).path).toBe("atlas/rig");
    expect(pushed.calib_version).toMatch(/^\d{8}T\d{6}Z-m$/);
    // The other sensor goes out exactly as stored (compared as the wire form).
    expect(pushed.sensors[0]).toEqual(JSON.parse(JSON.stringify(RIG.sensors[0])));
    expect(pushed.sensors[1]?.T_body_sensor[3]).toBeCloseTo(-0.2, 9);
  });
});
