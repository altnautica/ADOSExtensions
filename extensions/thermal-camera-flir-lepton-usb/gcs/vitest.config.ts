import { readFileSync } from "node:fs";

import { defineConfig } from "vitest/config";

// The bundle inlines plugin.css as text (esbuild `--loader:.css=text`); load
// it the same way under test.
export default defineConfig({
  plugins: [
    {
      name: "css-as-text",
      enforce: "pre",
      // Re-key the import so Vite's own CSS pipeline does not claim it.
      async resolveId(source, importer) {
        if (!source.endsWith(".css") || !importer) return null;
        const resolved = await this.resolve(source, importer, { skipSelf: true });
        return resolved ? `\0css-text:${resolved.id}.js` : null;
      },
      load(id) {
        if (!id.startsWith("\0css-text:")) return null;
        const file = id.slice("\0css-text:".length, -".js".length);
        return `export default ${JSON.stringify(readFileSync(file, "utf8"))};`;
      },
    },
  ],
  test: {
    include: ["tests/**/*.test.ts"],
  },
});
