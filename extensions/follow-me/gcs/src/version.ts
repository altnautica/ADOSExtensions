/**
 * The plugin version, read from the GCS package so it cannot drift from the
 * release (the manifest version is bumped with the package; a test checks
 * the two match).
 *
 * @license GPL-3.0-or-later
 */

import pkg from "../package.json";

export const PLUGIN_VERSION: string = pkg.version;
