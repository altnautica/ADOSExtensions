/**
 * `@altnautica/plugin-sdk/esbuild`: build tooling for inline GCS modules.
 *
 * An inline module runs in Mission Control's page and must use the host's
 * React, not a second copy: two Reacts in one tree break hooks and context.
 * `inlineSharedExternals()` rewrites the React entry points a module imports
 * to reads of the frozen `globalThis.__ADOS_INLINE_SHARED__` the host
 * publishes before importing the module:
 *
 *   react                  -> .react
 *   react-dom              -> .reactDom
 *   react-dom/client       -> .reactDomClient
 *   react/jsx-runtime      -> .jsxRuntime
 *   react/jsx-dev-runtime  -> built from .jsxRuntime
 *
 * ```js
 * import { build } from "esbuild";
 * import { inlineSharedExternals } from "@altnautica/plugin-sdk/esbuild";
 * await build({
 *   entryPoints: ["src/plugin.tsx"], bundle: true, format: "esm",
 *   jsx: "automatic", outfile: "world-engine.mjs",
 *   plugins: [inlineSharedExternals()],
 * });
 * ```
 *
 * The plugin is typed structurally so the SDK carries no esbuild dependency;
 * it is assignable to esbuild's `Plugin`.
 */

// Explicit extension: this entry runs in Node (a build script), whose ESM
// resolver does not add one.
import { INLINE_SHARED_GLOBAL } from "./inline.js";

/** The subset of esbuild's `PluginBuild` this plugin uses. */
export interface InlineSharedPluginBuild {
  onResolve(
    options: { filter: RegExp },
    callback: (args: { path: string }) => { path: string; namespace: string },
  ): void;
  onLoad(
    options: { filter: RegExp; namespace: string },
    callback: (args: { path: string }) => { contents: string; loader: "js" },
  ): void;
}

/** The esbuild plugin shape `inlineSharedExternals()` returns. */
export interface InlineSharedPlugin {
  name: string;
  setup(build: InlineSharedPluginBuild): void;
}

const NAMESPACE = "ados-inline-shared";

/** Module specifier -> the host-shared object that serves it. */
const SHARED_KEYS: Record<string, string> = {
  react: "react",
  "react-dom": "reactDom",
  "react-dom/client": "reactDomClient",
  "react/jsx-runtime": "jsxRuntime",
  "react/jsx-dev-runtime": "jsxRuntime",
};

/** CommonJS source for one shared module; esbuild turns named ESM imports of
 * it into property reads, so `import { useState } from "react"` works. */
function sharedModuleSource(specifier: string): string {
  const key = SHARED_KEYS[specifier];
  const read = [
    `var shared = globalThis[${JSON.stringify(INLINE_SHARED_GLOBAL)}];`,
    `if (!shared || !shared[${JSON.stringify(key)}]) {`,
    `  throw new Error(${JSON.stringify(`${specifier} is provided by the host; load this module through the Mission Control inline host`)});`,
    `}`,
  ];
  if (specifier === "react/jsx-dev-runtime") {
    // The dev transform calls jsxDEV(type, props, key, isStaticChildren);
    // serve it from the production runtime the host shares.
    return [
      ...read,
      `var rt = shared.jsxRuntime;`,
      `module.exports = { Fragment: rt.Fragment, jsxDEV: function (type, props, key, isStatic) {`,
      `  return (isStatic ? rt.jsxs : rt.jsx)(type, props, key);`,
      `} };`,
    ].join("\n");
  }
  return [...read, `module.exports = shared[${JSON.stringify(key)}];`].join("\n");
}

/** esbuild plugin: resolve the React entry points to the host's shared copy. */
export function inlineSharedExternals(): InlineSharedPlugin {
  const filter = new RegExp(
    `^(${Object.keys(SHARED_KEYS)
      .map((s) => s.replace(/[/-]/g, "\\$&"))
      .join("|")})$`,
  );
  return {
    name: "ados-inline-shared-externals",
    setup(build) {
      build.onResolve({ filter }, (args) => ({ path: args.path, namespace: NAMESPACE }));
      build.onLoad({ filter: /.*/, namespace: NAMESPACE }, (args) => ({
        contents: sharedModuleSource(args.path),
        loader: "js",
      }));
    },
  };
}
