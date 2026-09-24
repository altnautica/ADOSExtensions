/** A module imported as `./file.ts?worker-source`: the build bundles the file
 * separately and hands back its source text (see `build.mjs`). */
declare module "*?worker-source" {
  const source: string;
  export default source;
}
