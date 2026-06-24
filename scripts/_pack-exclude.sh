# Shared packaging helpers, sourced by pack.sh and pack-rust.sh.
#
# Single source of truth for two things that must never drift between the
# two packers:
#   1. PACK_RSYNC_EXCLUDES — what never ships inside a .adosplug. A published
#      archive is public, so raw source, tests, build caches, and tooling
#      config stay out of it.
#   2. assert_entrypoint_in_stage — the loud guard that stops a half-built
#      archive (a manifest declaring gcs/plugin.bundle.js or a rust binary that
#      the build step never produced) from being zipped and published. A
#      missing entrypoint is a build failure, not something to discover at
#      install time when the iframe or the supervisor cannot find it.
#
# shellcheck shell=bash

# Names excluded from every .adosplug. rsync matches each name at any depth.
# A rust packer adds 'agent' and 'target' to this on top (the compiled binary
# is staged separately; the crate source never ships).
PACK_RSYNC_EXCLUDES=(
  --exclude 'dist'
  --exclude 'node_modules'
  --exclude '.venv'
  --exclude '.pnpm-store'
  --exclude 'tests'
  --exclude '__pycache__'
  --exclude '.pytest_cache'
  --exclude '.mypy_cache'
  --exclude '.ruff_cache'
  --exclude '.tsbuildinfo'
  --exclude '*.tsbuildinfo'
  --exclude 'tsconfig.json'
  --exclude 'vitest.config.ts'
  --exclude 'esbuild.config.*'
  --exclude '*.egg-info'
  --exclude 'src'
  --exclude 'package.json'
)

# Read the entrypoint declared inside the manifest's `gcs:` block. The gcs
# block is indented under a column-0 `gcs:` key; a column-0 line ends it. Prints
# nothing when the manifest has no gcs half.
manifest_gcs_entrypoint() {
  awk '
    /^gcs:/ { in_gcs = 1; next }
    in_gcs && /^[A-Za-z]/ { in_gcs = 0 }
    in_gcs && /entrypoint:/ {
      sub(/.*entrypoint:[[:space:]]*/, "")
      gsub(/"/, "")
      sub(/[[:space:]]+#.*/, "")
      sub(/[[:space:]]*$/, "")
      print
      exit
    }
  ' "$1"
}

# Fail loudly if a manifest-declared entrypoint is absent from the staged tree.
# Call AFTER the build + rsync stage and BEFORE the zip.
#   assert_entrypoint_in_stage <stage_dir> <relative_path> <label>
assert_entrypoint_in_stage() {
  local stage="$1" rel="$2" label="$3"
  [[ -z "${rel}" ]] && return 0
  if [[ ! -f "${stage}/${rel}" ]]; then
    echo "FATAL: manifest declares ${label} '${rel}', but it is not in the packed archive." >&2
    echo "  This means the build step did not produce it. Never publish a half-archive:" >&2
    echo "  build the GCS bundle (pnpm build) and/or the agent binary before packing." >&2
    exit 1
  fi
}
