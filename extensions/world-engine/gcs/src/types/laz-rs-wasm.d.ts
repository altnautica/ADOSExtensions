/**
 * The laz-rs decoder bundled inside `@loaders.gl/las` (not an exported subpath).
 * `build.mjs` aliases these ids to the package's own files, so the viewer
 * initialises the same module instance the loader uses.
 */
declare module "laz-rs-wasm" {
  export function initSync(module: { module: BufferSource | WebAssembly.Module }): unknown;
}

/** The decoder's wasm, bundled as bytes. */
declare module "laz-rs-wasm/bytes" {
  const bytes: Uint8Array;
  export default bytes;
}
