import type { FirmwareType } from "./types";

/** The plugin config shape the tab reads: ``firmware`` is an object
 * with a ``type`` key, per ``config-schema.json``. */
export interface VisionNavConfig {
  firmware?: { type?: unknown } | null;
}

const FIRMWARE_TYPES: readonly FirmwareType[] = ["ardupilot", "px4", "betaflight", "inav"];

/** The firmware named by a plugin config, or ``null`` when absent or
 * not one the tab knows. */
export function firmwareFromConfig(
  next: VisionNavConfig | null | undefined,
): FirmwareType | null {
  const type = next?.firmware?.type;
  return typeof type === "string" && (FIRMWARE_TYPES as readonly string[]).includes(type)
    ? (type as FirmwareType)
    : null;
}
