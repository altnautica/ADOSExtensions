import { afterEach } from "vitest";
import { cleanup } from "@testing-library/react";

// Auto-unmount any rendered component between tests so the global
// document tree doesn't accumulate stale nodes (which would cause
// "multiple elements found by testid" failures in subsequent tests).
afterEach(() => {
  cleanup();
});
