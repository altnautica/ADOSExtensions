#!/usr/bin/env bash
# Build the self-contained KISS-ICP runtime of the lidar-slam service for one
# architecture: a relocatable CPython (python-build-standalone, pinned with its
# sha256) whose own site-packages hold the wheels the `kiss_icp_live` project
# locks (python/kiss_icp_live/uv.lock: versions and sha256 hashes), so the
# odometry runs offline from the install tree with no `uv sync` on the node.
#
# Usage: build-runtime.sh --arch x86_64|aarch64 [--out DIR] [--uv UV]
#                         [--cache DIR] [--python PY]
#
# Output, under DIR (default ./payload):
#   kiss-icp-runtime-<arch>-linux/       the runtime tree (bin/python3, lib/...)
#   kiss-icp-runtime-<arch>-linux.zip    the same tree, the release payload the
#                                        host unpacks into
#                                        lib/<arch>-linux/kiss-icp-runtime/
# and one line with the zip's sha256 and size, for the manifest payloads.
#
# Inside the tree: requirements.lock.txt (the exported lock the wheels were
# installed from, with hashes) and LICENSES.txt (CPython's licence and every
# installed distribution's licence files). Licences: CPython PSF-2.0,
# kiss-icp MIT, numpy BSD-3-Clause (its bundled OpenBLAS BSD-3, libgfortran
# GPL-3.0 with the GCC runtime library exception), pydantic MIT;
# the full list is LICENSES.txt.
#
# Wheels are resolved for `<arch>-manylinux_2_28` and binary only, so nothing
# in the tree needs a glibc newer than 2.28 (the floor of the release's other
# native payloads); the build checks every ELF file's GLIBC_ symbol versions
# against that floor with objdump. Bytecode is compiled with unchecked hashes
# so the read-only tree never needs a .pyc rewrite; `--python` names an
# interpreter of the same CPython minor version to compile with when the
# runtime's own cannot run on this host (a cross-arch build).
#
# Release: each leg of .github/workflows/navigation-release.yml runs
# `build-runtime.sh --arch <leg arch>` natively and stages the zip as the
# asset named in scripts/navigation-payloads.tsv.
set -euo pipefail

# python-build-standalone release and its install_only_stripped archives.
PBS_TAG="20261110"
PY_VERSION="3.12.16"
PY_MINOR="3.12"
declare -A PBS_SHA256=(
  [x86_64]=6a9e62dad033790aef722dbd370e0ce76ce17b73627ba3ae679f537049350b39
  [aarch64]=b6a41defec4975d94b4ff1950236e7383e5c02ba40ea48c9ec33c66087a35aab
)
GLIBC_FLOOR="2.28"

ARCH=""
OUT="$(pwd)/payload"
UV="uv"
CACHE=""
PY_BUILD=""

while [ $# -gt 0 ]; do
  case "$1" in
    --arch) shift; ARCH="${1:?--arch needs x86_64 or aarch64}" ;;
    --out) shift; OUT="${1:?--out needs a directory}" ;;
    --uv) shift; UV="${1:?--uv needs a path}" ;;
    --cache) shift; CACHE="${1:?--cache needs a directory}" ;;
    --python) shift; PY_BUILD="${1:?--python needs an interpreter}" ;;
    -h|--help) sed -n '2,34p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

case "$ARCH" in
  x86_64|aarch64) ;;
  *) echo "--arch must be x86_64 or aarch64" >&2; exit 2 ;;
esac
[ "$(uname -s)" = "Linux" ] || { echo "the runtime builds on Linux only" >&2; exit 2; }
for tool in curl tar sha256sum objdump python3 "$UV"; do
  command -v "$tool" >/dev/null || { echo "missing tool $tool" >&2; exit 2; }
done

HERE="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$HERE/../python/kiss_icp_live"
NAME="kiss-icp-runtime-$ARCH-linux"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
CACHE="${CACHE:-$WORK}"
mkdir -p "$CACHE"

# 1. The interpreter, checksum-verified before it is unpacked.
TGZ="cpython-${PY_VERSION}+${PBS_TAG}-${ARCH}-unknown-linux-gnu-install_only_stripped.tar.gz"
if ! echo "${PBS_SHA256[$ARCH]}  $CACHE/$TGZ" | sha256sum -c --status - 2>/dev/null; then
  curl -fsSLo "$CACHE/$TGZ" \
    "https://github.com/astral-sh/python-build-standalone/releases/download/${PBS_TAG}/${TGZ/+/%2B}"
  echo "${PBS_SHA256[$ARCH]}  $CACHE/$TGZ" | sha256sum -c -
fi
tar -xzf "$CACHE/$TGZ" -C "$WORK"
ROOT="$WORK/python"
STDLIB="$ROOT/lib/python$PY_MINOR"
SITE="$STDLIB/site-packages"

# 2. Drop what the odometry never loads: the GUI toolkit, the test suite,
#    development headers, pip, every console script, and the shared libpython
#    (the interpreter links CPython statically; it is there for embedders).
rm -rf "$ROOT/include" "$ROOT/share" "$ROOT/lib/pkgconfig" \
  "$ROOT"/lib/tcl* "$ROOT"/lib/tk* "$ROOT"/lib/itcl* "$ROOT"/lib/thread* \
  "$ROOT"/lib/libtcl* "$ROOT"/lib/libtk* "$ROOT"/lib/libpython* \
  "$STDLIB"/{test,idlelib,tkinter,turtledemo,ensurepip,lib2to3,pydoc_data} \
  "$STDLIB/turtle.py" "$STDLIB"/config-"$PY_MINOR"-* "$STDLIB"/lib-dynload/_tkinter* \
  "$SITE"/pip "$SITE"/pip-*
# One real interpreter file: the archive carries no symlinks.
mv "$ROOT/bin/python$PY_MINOR" "$WORK/python3"
rm -rf "$ROOT/bin"
mkdir "$ROOT/bin"
mv "$WORK/python3" "$ROOT/bin/python3"
if objdump -p "$ROOT/bin/python3" | grep -q 'NEEDED.*libpython'; then
  echo "ERROR: this interpreter loads a shared libpython, which the runtime drops" >&2
  exit 1
fi

# 3. The locked wheels, binary only, for this arch at the glibc floor.
"$UV" export --project "$PROJECT" --frozen --no-dev --no-emit-project --no-header \
  --format requirements-txt --output-file "$ROOT/requirements.lock.txt" >/dev/null
UV_NO_CONFIG=1 "$UV" pip install --no-config --target "$SITE" --no-deps --require-hashes \
  --only-binary :all: --python-version "$PY_MINOR" \
  --python-platform "$ARCH-manylinux_${GLIBC_FLOOR/./_}" \
  --no-compile -r "$ROOT/requirements.lock.txt"
# Console scripts carry this host's paths; test suites, stubs and C headers
# never load.
rm -rf "$SITE/bin" "$SITE/numpy/_core/include"
find "$SITE/numpy" -depth -type d -name tests -exec rm -rf {} +
find "$SITE" -type f \( -name '*.pyi' -o -name '*.pxd' -o -name '*.pyx' \) -delete
find "$ROOT" -type l -delete

# 4. Bytecode for the read-only tree: unchecked hashes, so no import ever
#    revalidates (or tries to rewrite) a .pyc against the unpacked mtimes.
find "$ROOT" -type d -name __pycache__ -prune -exec rm -rf {} +
if [ -z "$PY_BUILD" ]; then
  PY_BUILD="$ROOT/bin/python3"
fi
"$PY_BUILD" -c "import sys; assert sys.version_info[:2] == tuple(map(int, '$PY_MINOR'.split('.'))), sys.version" \
  || { echo "--python must be CPython $PY_MINOR" >&2; exit 1; }
"$PY_BUILD" -E -s -m compileall -q -j 0 --invalidation-mode unchecked-hash \
  -s "$ROOT/" -p / "$STDLIB" >/dev/null

# 5. Licences: CPython's, then every installed distribution's licence files.
{
  printf '==== CPython %s (python-build-standalone %s) ====\n\n' "$PY_VERSION" "$PBS_TAG"
  cat "$STDLIB/LICENSE.txt"
  for dist in "$SITE"/*.dist-info; do
    find "$dist" -type f \( -iname 'LICEN[CS]E*' -o -iname 'COPYING*' -o -iname 'NOTICE*' \) | sort |
      while read -r f; do
        printf '\n\n==== %s: %s ====\n\n' "$(basename "$dist" .dist-info)" "${f#"$dist"/}"
        cat "$f"
      done
  done
} > "$ROOT/LICENSES.txt"

# 6. No file may need a glibc above the floor.
need="$(find "$ROOT" -type f \( -name '*.so*' -o -path "$ROOT/bin/python3" \) -print0 |
  xargs -0 objdump -T 2>/dev/null | grep -o 'GLIBC_[0-9.]*' | sed 's/GLIBC_//' | sort -uV | tail -n1)"
echo "KISS-ICP runtime ($ARCH-linux) needs glibc ${need:-none}"
if [ -n "$need" ] && [ "$(printf '%s\n%s\n' "$need" "$GLIBC_FLOOR" | sort -V | tail -n1)" != "$GLIBC_FLOOR" ]; then
  echo "ERROR: the runtime needs glibc $need, above the $GLIBC_FLOOR floor" >&2
  exit 1
fi

# 7. The tree and its zip: sorted members, fixed timestamps, regular files only.
rm -rf "${OUT:?}/$NAME" "$OUT/$NAME.zip"
mv "$ROOT" "$OUT/$NAME"
# Writable by the owner alone, as the lidar-slam service requires of a runtime
# it runs (an installed tree gets the same modes from the host).
chmod -R u+rwX,go-w "$OUT/$NAME"
python3 - "$OUT/$NAME" "$OUT/$NAME.zip" <<'PY'
import os
import stat
import sys
import zipfile

root, out = sys.argv[1], sys.argv[2]
members = []
for d, dirs, files in os.walk(root):
    dirs.sort()
    for f in sorted(files):
        p = os.path.join(d, f)
        st = os.lstat(p)
        if not stat.S_ISREG(st.st_mode):
            sys.exit(f"ERROR: {p} is not a regular file")
        members.append((os.path.relpath(p, root), p, st.st_mode & 0o111))
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
    for rel, path, exe in sorted(members):
        info = zipfile.ZipInfo(rel, date_time=(1980, 1, 1, 0, 0, 0))
        info.compress_type = zipfile.ZIP_DEFLATED
        info.external_attr = (stat.S_IFREG | (0o755 if exe else 0o644)) << 16
        with open(path, "rb") as fh:
            z.writestr(info, fh.read(), compresslevel=9)
print(f"{len(members)} files")
PY
printf '%s  sha256=%s  size_bytes=%s  unpacked_bytes=%s\n' "$NAME.zip" \
  "$(sha256sum "$OUT/$NAME.zip" | cut -d' ' -f1)" "$(stat -c %s "$OUT/$NAME.zip")" \
  "$(du -sb "$OUT/$NAME" | cut -f1)"
