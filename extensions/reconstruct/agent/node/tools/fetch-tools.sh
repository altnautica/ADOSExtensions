#!/usr/bin/env bash
# Install the pinned reconstruction tools listed in tool-pins.json (Linux/macOS).
#
#   archive : download, verify sha256, extract to <tools-dir>/<name>/<version>/
#   npm     : npm install --prefix <tools-dir>/<name>/<version> <package>@<version>
#   cargo   : cargo install --git <repo> --rev <rev> --locked --root <tools-dir>/<name>/<version> <package>
#   uv      : uv sync --project <node>/<project>
#   docker  : docker pull <image> (or docker build from a pinned commit) when docker is reachable
# Every installed tool is probed afterwards and reported OK or FAIL.
#
# Usage: fetch-tools.sh [--list] [--tool name[,name...]]... [--force] [--tools-dir PATH]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_DIR="$(dirname "$SCRIPT_DIR")"
PINS="$SCRIPT_DIR/tool-pins.json"
TOOLS_DIR="$SCRIPT_DIR/cache"
LIST=0
FORCE=0
SELECTED=()

while [ $# -gt 0 ]; do
  case "$1" in
    --list|-l) LIST=1 ;;
    --force|-f) FORCE=1 ;;
    --tool|-t) shift; IFS=',' read -r -a parts <<<"${1:?--tool needs a name}"; SELECTED+=("${parts[@]}") ;;
    --tools-dir) shift; TOOLS_DIR="${1:?--tools-dir needs a path}" ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

case "$(uname -s)-$(uname -m)" in
  Linux-x86_64) PLATFORM=x86_64-linux ;;
  Linux-aarch64|Linux-arm64) PLATFORM=aarch64-linux ;;
  Darwin-arm64) PLATFORM=aarch64-macos ;;
  Darwin-x86_64) PLATFORM=x86_64-macos ;;
  *) echo "unsupported host $(uname -s)-$(uname -m)" >&2; exit 2 ;;
esac

# One tab-separated row per tool; "-" marks an empty field, probe args are
# joined with \x1f.
FIELDS_PY='
import json, sys
pins = json.load(open(sys.argv[1])); plat = sys.argv[2]
for name, t in pins["tools"].items():
    p = t.get("platforms", {}).get(plat, {})
    b = t.get("build", {}); c = t.get("cargo", {})
    row = [name, t["version"], t["kind"], p.get("url"), p.get("sha256"), p.get("archive"), p.get("inner"),
           p.get("bin") or t.get("bin"), t.get("package"), t.get("project"), t.get("image"),
           b.get("repo"), b.get("commit"), b.get("dockerfile"), b.get("tag"),
           c.get("repo"), c.get("rev"), c.get("package"), "\x1f".join(t.get("probe", [])),
           "yes" if p else "no"]
    print("\t".join(x if x else "-" for x in row))
'
FIELDS_JS='
const fs = require("fs"); const [pinsPath, plat] = process.argv.slice(1);
const pins = JSON.parse(fs.readFileSync(pinsPath, "utf8"));
for (const [name, t] of Object.entries(pins.tools)) {
  const p = (t.platforms || {})[plat] || {}; const b = t.build || {}; const c = t.cargo || {};
  const row = [name, t.version, t.kind, p.url, p.sha256, p.archive, p.inner, p.bin || t.bin, t.package,
    t.project, t.image, b.repo, b.commit, b.dockerfile, b.tag, c.repo, c.rev, c.package,
    (t.probe || []).join("\x1f"), Object.keys(p).length ? "yes" : "no"];
  console.log(row.map((x) => (x ? x : "-")).join("\t"));
}
'
pin_rows() {
  if command -v python3 >/dev/null 2>&1; then python3 -c "$FIELDS_PY" "$PINS" "$PLATFORM"
  elif command -v node >/dev/null 2>&1; then node -e "$FIELDS_JS" "$PINS" "$PLATFORM"
  else echo "need python3 or node to read tool-pins.json" >&2; exit 2
  fi
}

find_uv() {
  if command -v uv >/dev/null 2>&1; then command -v uv; return; fi
  for p in "$HOME/.local/bin/uv" "$HOME/.cargo/bin/uv"; do [ -x "$p" ] && { echo "$p"; return; }; done
  return 1
}

DOCKER_STATE=unknown
docker_ok() {
  if [ "$DOCKER_STATE" = unknown ]; then
    if command -v docker >/dev/null 2>&1 && docker version >/dev/null 2>&1; then DOCKER_STATE=yes; else DOCKER_STATE=no; fi
  fi
  [ "$DOCKER_STATE" = yes ]
}

sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi
}

val() { [ "$1" = "-" ] && echo "" || echo "$1"; }

load_row() {
  IFS=$'\t' read -r NAME VERSION KIND URL SHA ARCHIVE INNER BIN PACKAGE PROJECT IMAGE \
    B_REPO B_COMMIT B_FILE B_TAG C_REPO C_REV C_PKG PROBE HASPLAT <<<"$1"
  DIR="$TOOLS_DIR/$NAME/$VERSION"
  DOCKER_REF="$IMAGE"; [ "$B_TAG" != "-" ] && DOCKER_REF="$B_TAG"
  PROBE_ARGS=()
  if [ "$PROBE" != "-" ]; then IFS=$'\x1f' read -r -a PROBE_ARGS <<<"$PROBE"; fi
}

installed_state() {
  case "$KIND" in
    uv) [ -d "$NODE_DIR/$PROJECT/.venv" ] && echo installed || echo missing ;;
    docker)
      if ! docker_ok; then echo "unknown (docker unavailable)"
      elif docker image inspect "$DOCKER_REF" >/dev/null 2>&1; then echo installed
      else echo missing; fi ;;
    *) [ "$BIN" != "-" ] && [ -e "$DIR/$BIN" ] && echo installed || echo missing ;;
  esac
}

platform_label() {
  case "$KIND" in
    docker) echo "linux/amd64 container" ;;
    npm) echo "any (node)" ;;
    uv) echo "any (uv)" ;;
    cargo) echo "any (cargo)" ;;
    *) [ "$HASPLAT" = yes ] && echo "$PLATFORM" || echo "$PLATFORM unavailable" ;;
  esac
}

extract() { # file kind dest
  mkdir -p "$3"
  case "$2" in
    zip) if command -v unzip >/dev/null 2>&1; then unzip -q -o "$1" -d "$3"; else tar -xf "$1" -C "$3"; fi ;;
    tar.gz) tar -xzf "$1" -C "$3" ;;
    tar.xz) tar -xJf "$1" -C "$3" ;;
    7z) 7z x -y "-o$3" "$1" >/dev/null ;;
    exe) cp "$1" "$3/" ;;
    *) echo "unknown archive kind $2" >&2; return 1 ;;
  esac
}

install_archive() {
  if [ "$URL" = "-" ]; then echo "  skipped (no $PLATFORM build pinned)"; return 2; fi
  local dl="$TOOLS_DIR/_downloads/$NAME-$VERSION-$(basename "$URL")"
  mkdir -p "$TOOLS_DIR/_downloads"
  if [ "$FORCE" = 1 ] || [ ! -f "$dl" ] || [ "$(sha256_of "$dl")" != "$SHA" ]; then
    echo "  downloading $URL"
    rm -f "$dl"
    curl -L --fail --retry 3 -o "$dl" "$URL"
  fi
  local got; got="$(sha256_of "$dl")"
  if [ "$got" != "$SHA" ]; then rm -f "$dl"; echo "  sha256 mismatch: got $got, pinned $SHA" >&2; return 1; fi
  echo "  sha256 OK"
  local tmp="$DIR.partial"
  rm -rf "$tmp"
  extract "$dl" "$ARCHIVE" "$tmp"
  if [ "$INNER" != "-" ]; then extract "$tmp/$INNER" zip "$tmp"; rm -f "$tmp/$INNER"; fi
  rm -rf "$DIR"
  mv "$tmp" "$DIR"
  [ "$BIN" != "-" ] && chmod +x "$DIR/$BIN" 2>/dev/null || true
}

install_npm() {
  command -v npm >/dev/null 2>&1 || { echo "  skipped (npm unavailable)"; return 2; }
  mkdir -p "$DIR"
  npm install --prefix "$DIR" --no-audit --no-fund --loglevel=error "$PACKAGE@$VERSION"
}

install_cargo() {
  command -v cargo >/dev/null 2>&1 || { echo "  skipped (cargo unavailable)"; return 2; }
  local extra=(); [ "$FORCE" = 1 ] && extra=(--force)
  cargo install --git "$C_REPO" --rev "$C_REV" --locked --root "$DIR" ${extra[@]+"${extra[@]}"} "$C_PKG"
}

install_uv() {
  local uv; uv="$(find_uv)" || { echo "  skipped (uv unavailable)"; return 2; }
  "$uv" sync --project "$NODE_DIR/$PROJECT"
}

install_docker() {
  docker_ok || { echo "  skipped (docker unavailable)"; return 2; }
  if [ "$B_REPO" != "-" ]; then
    docker build -t "$B_TAG" -f "$B_FILE" "$B_REPO.git#$B_COMMIT"
  else
    docker pull "$IMAGE"
  fi
}

run_probe() {
  case "$KIND" in
    uv) (cd "$NODE_DIR/$PROJECT" && "$(find_uv)" run --project . ${PROBE_ARGS[@]+"${PROBE_ARGS[@]}"}) >/dev/null 2>&1 ;;
    docker) docker image inspect "$DOCKER_REF" >/dev/null 2>&1 ;;
    *) [ -x "$DIR/$BIN" ] && (cd "$DIR" && "$DIR/$BIN" ${PROBE_ARGS[@]+"${PROBE_ARGS[@]}"}) >/dev/null 2>&1 ;;
  esac
}

ROWS="$(pin_rows)"

if [ "$LIST" = 1 ]; then
  printf '%-16s %-24s %-8s %-24s %s\n' NAME VERSION KIND PLATFORM STATE
  while IFS= read -r row; do
    load_row "$row"
    printf '%-16s %-24s %-8s %-24s %s\n' "$NAME" "$VERSION" "$KIND" "$(platform_label)" "$(installed_state)"
  done <<<"$ROWS"
  exit 0
fi

if [ ${#SELECTED[@]} -gt 0 ]; then
  known=" $(cut -f1 <<<"$ROWS" | tr '\n' ' ') "
  for t in "${SELECTED[@]}"; do
    case "$known" in *" $t "*) ;; *) echo "unknown tool: $t (known:$known)" >&2; exit 2 ;; esac
  done
fi

failed=0
while IFS= read -r row; do
  load_row "$row"
  if [ ${#SELECTED[@]} -gt 0 ]; then
    case " ${SELECTED[*]} " in *" $NAME "*) ;; *) continue ;; esac
  fi
  echo "== $NAME $VERSION [$KIND]"
  rc=0
  if [ "$KIND" != uv ] && [ "$FORCE" = 0 ] && [ "$(installed_state)" = installed ]; then
    echo "  already installed"
  else
    "install_$KIND" || rc=$?
  fi
  if [ "$rc" = 2 ]; then continue; fi
  if [ "$rc" != 0 ]; then echo "  FAIL: install step exited $rc"; failed=$((failed + 1)); continue; fi
  if run_probe; then echo "  probe OK"; else echo "  probe FAIL"; failed=$((failed + 1)); fi
done <<<"$ROWS"

[ "$failed" = 0 ]
