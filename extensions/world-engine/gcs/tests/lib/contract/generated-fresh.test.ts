// @vitest-environment node
/**
 * The committed contract types in `src/lib/contract/generated/` must be
 * exactly what `scripts/gen-contract.mjs` produces from the contract schema
 * and proto files today. Regenerate with `node scripts/gen-contract.mjs`.
 */

import { execFileSync } from "node:child_process";
import { mkdtempSync, readdirSync, readFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";
import { describe, expect, it } from "vitest";

const gcsDir = resolve(__dirname, "../../..");
const committedDir = join(gcsDir, "src/lib/contract/generated");

function listFiles(dir: string, prefix = ""): string[] {
  return readdirSync(dir, { withFileTypes: true })
    .flatMap((e) =>
      e.isDirectory() ? listFiles(join(dir, e.name), `${prefix}${e.name}/`) : [`${prefix}${e.name}`],
    )
    .sort();
}

describe("generated capture contract types", () => {
  it("match a fresh run of the generator", () => {
    const out = mkdtempSync(join(tmpdir(), "ados-contract-"));
    try {
      execFileSync(process.execPath, [join(gcsDir, "scripts/gen-contract.mjs"), "--out", out], {
        cwd: gcsDir,
        stdio: "pipe",
      });
      const fresh = listFiles(out);
      expect(listFiles(committedDir)).toEqual(fresh);
      for (const file of fresh) {
        const want = readFileSync(join(out, file), "utf8");
        const have = readFileSync(join(committedDir, file), "utf8");
        expect(have, `${file} is stale; run node scripts/gen-contract.mjs`).toBe(want);
      }
    } finally {
      rmSync(out, { recursive: true, force: true });
    }
  }, 120_000);
});
