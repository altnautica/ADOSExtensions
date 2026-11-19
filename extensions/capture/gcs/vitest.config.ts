import { fileURLToPath } from "node:url";
import { defineConfig } from "vitest/config";

const mappingUi = fileURLToPath(new URL("../../../packages/mapping-ui/src/", import.meta.url));

export default defineConfig({
  resolve: {
    alias: [{ find: /^@altnautica\/mapping-ui\/(.*)$/, replacement: `${mappingUi}$1` }],
  },
  test: {
    environment: "happy-dom",
    include: ["tests/**/*.test.{ts,tsx}"],
  },
});
