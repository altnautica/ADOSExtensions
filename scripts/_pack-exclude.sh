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

# Names excluded from every .adosplug. Patterns WITHOUT a leading slash match
# each name at any depth; patterns WITH a leading slash are anchored to the
# extension root. The GCS TypeScript sources and its build tooling are stripped
# by anchored path, because the agent half's Python package also lives under a
# `src` directory (`agent/src/<module>/`) and must ship: it is the code the
# supervisor imports at the manifest's agent.entrypoint.
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
  --exclude '*.egg-info'
  --exclude 'esbuild.config.*'
  --exclude '/gcs/src'
  --exclude '/gcs/build.mjs'
  --exclude '/gcs/package.json'
  --exclude '/gcs/tsconfig.json'
  --exclude '/gcs/vitest.config.ts'
  --exclude '/package.json'
  --exclude '/tsconfig.json'
  --exclude '/vitest.config.ts'
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

# Read a scalar field declared inside the manifest's `agent:` block. The agent
# block is indented under a column-0 `agent:` key; a column-0 line ends it.
# Prints nothing when the manifest has no agent half.
#   manifest_agent_field <manifest> <field>
manifest_agent_field() {
  awk -v field="$2" '
    /^agent:/ { in_agent = 1; next }
    in_agent && /^[A-Za-z]/ { in_agent = 0 }
    in_agent && $0 ~ ("^[[:space:]]+" field ":") {
      sub(".*" field ":[[:space:]]*", "")
      gsub(/"/, "")
      sub(/[[:space:]]+#.*/, "")
      sub(/[[:space:]]*$/, "")
      print
      exit
    }
  ' "$1"
}

manifest_agent_runtime() { manifest_agent_field "$1" "runtime"; }
manifest_agent_entrypoint() { manifest_agent_field "$1" "entrypoint"; }

# Fail loudly when a manifest declares a python agent entrypoint whose module is
# absent from the staged tree. This is the guard that catches a half-archive:
# a manifest + locales + GCS bundle with no agent source installs cleanly, the
# signature verifies, and the supervisor then dies importing a module that was
# never packed.
#
# The entrypoint is `dotted.module.path:ClassName`; the module resolves to one of
# `agent/src/<path>.py`, `agent/src/<path>/__init__.py`, `agent/<path>.py` or
# `agent/<path>/__init__.py` (both the src-layout and flat-package conventions
# in this repo).
#   assert_agent_entrypoint_in_stage <stage_dir> <manifest>
assert_agent_entrypoint_in_stage() {
  local stage="$1" manifest="$2"
  local runtime entrypoint module rel candidate
  runtime="$(manifest_agent_runtime "${manifest}")"
  [[ "${runtime}" != "python" ]] && return 0

  entrypoint="$(manifest_agent_entrypoint "${manifest}")"
  if [[ -z "${entrypoint}" ]]; then
    echo "FATAL: manifest declares 'agent.runtime: python' with no agent.entrypoint." >&2
    exit 1
  fi

  module="${entrypoint%%:*}"
  rel="${module//.//}"
  for candidate in \
    "agent/src/${rel}.py" \
    "agent/src/${rel}/__init__.py" \
    "agent/${rel}.py" \
    "agent/${rel}/__init__.py"; do
    if [[ -f "${stage}/${candidate}" ]]; then
      echo "  agent.entrypoint '${entrypoint}' resolves to ${candidate}"
      return 0
    fi
  done

  echo "FATAL: manifest declares agent.entrypoint '${entrypoint}', but the module" >&2
  echo "  it names is not in the packed archive. Never publish a half-archive:" >&2
  echo "  an archive with no agent source installs and verifies, then the" >&2
  echo "  supervisor dies on ModuleNotFoundError at first start." >&2
  echo "  Looked for, relative to the archive root:" >&2
  echo "    agent/src/${rel}.py" >&2
  echo "    agent/src/${rel}/__init__.py" >&2
  echo "    agent/${rel}.py" >&2
  echo "    agent/${rel}/__init__.py" >&2
  exit 1
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
