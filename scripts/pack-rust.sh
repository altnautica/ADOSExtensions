#!/usr/bin/env bash
#
# Pack a Rust extension into an unsigned .adosplug archive.
#
# Usage:
#   scripts/pack-rust.sh <extension-folder-or-path> [target]
#   scripts/pack-rust.sh <extension-folder-or-path> --payload-manifest <list> [--arch-os <a,b>]
#
# Examples:
#   scripts/pack-rust.sh _hello-rust
#   scripts/pack-rust.sh _hello-rust aarch64-unknown-linux-musl
#   scripts/pack-rust.sh ./extensions/_hello-rust
#   scripts/pack-rust.sh world-engine --payload-manifest payloads/payloads.tsv
#
# Binary mode (the default) is for a single-binary extension:
#   1. Reads the manifest's agent.entrypoint, which for a rust runtime is the
#      path the binary takes inside the archive (e.g. bin/hello-rust).
#   2. Cross-compiles the crate for the target via scripts/build-rust.sh.
#   3. Stages the manifest, the compiled binary at agent.entrypoint, and any
#      assets (locales, config-schema.json, README, gcs bundle if present).
#   4. Zips the staging tree into dist/<plugin-id>-<version>.adosplug.
#
# Payload mode (--payload-manifest) is for an extension whose binaries and
# large assets ship as install-time payloads (manifest `agent.payloads`)
# instead of inside the archive. It builds nothing: the caller has already
# built the GCS bundle and every payload file. It:
#   1. Reads the asset list: one whitespace-separated row per payload,
#        <name> <path-in-archive> <arch_os|-> <profiles|-> <url>
#      where <name> is the file in the list's own directory, <profiles> is a
#      comma-separated profile list, and `-` leaves the field unset (fetched on
#      every arch / every profile). Blank lines and `#` comments are ignored.
#   2. Stages the extension tree without the Rust sources, keeping the runtime
#      helper scripts under agent/python/, and without any payload path.
#   3. Writes each payload's source, sha256 and size_bytes into the STAGED
#      manifest's agent.payloads block (the source manifest is never touched).
#   4. Refuses to pack when a `bin:` reference names no `binaries` entry, when
#      a `binaries` path for a covered arch has no payload, or when a payload
#      breaks the host's rules (https source on the download allowlist, size
#      1 byte to 1 GiB, a relative path).
#   --arch-os limits the covered arches (default: every arch_os named in the
#   manifest's `binaries`); a local bench build passes only its own arch.
#
# Signing is the separate, final step and is unchanged across runtimes:
#   ADOS_SIGNING_KEY=/path/to/key.ed25519 scripts/sign.sh dist/<plugin>.adosplug
#
# The signature covers every archive entry, so the binary at agent.entrypoint
# is protected by the same canonical-payload-hash signature as the manifest. A
# payload is covered through the manifest: its sha256 and size are part of the
# signed manifest, and the host refuses a fetched file that does not match.

set -euo pipefail

usage() {
  echo "usage: pack-rust.sh <extension-folder-or-path> [target]" >&2
  echo "       pack-rust.sh <extension-folder-or-path> --payload-manifest <list> [--arch-os <a,b>]" >&2
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

arg="$1"
shift
target="aarch64-unknown-linux-musl"
payload_list=""
payload_arch_os=""
if [[ "${1:-}" == --* ]]; then
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --payload-manifest)
        payload_list="${2:?--payload-manifest needs a file}"
        shift 2
        ;;
      --arch-os)
        payload_arch_os="${2:?--arch-os needs a comma-separated list}"
        shift 2
        ;;
      *)
        usage
        exit 2
        ;;
    esac
  done
  if [[ -z "${payload_list}" ]]; then
    echo "--arch-os applies only with --payload-manifest" >&2
    exit 2
  fi
  if [[ ! -f "${payload_list}" ]]; then
    echo "payload list not found: ${payload_list}" >&2
    exit 2
  fi
  payload_list="$(cd "$(dirname "${payload_list}")" && pwd)/$(basename "${payload_list}")"
elif [[ $# -gt 0 ]]; then
  target="$1"
fi
repo_root="$(cd "$(dirname "$0")/.." && pwd)"

# Shared exclude list + the entrypoint-existence guard (see _pack-exclude.sh).
source "$(dirname "$0")/_pack-exclude.sh"

# Resolve the extension directory the same way pack.sh does.
if [[ -d "${arg}" && -f "${arg}/manifest.yaml" ]]; then
  ext_dir="$(cd "${arg}" && pwd)"
elif [[ -d "${repo_root}/extensions/${arg}" ]]; then
  ext_dir="${repo_root}/extensions/${arg}"
else
  echo "extension not found: ${arg}" >&2
  echo "  tried: ${arg} and ${repo_root}/extensions/${arg}" >&2
  exit 2
fi

manifest_src="${ext_dir}/manifest.yaml"
if [[ ! -f "${manifest_src}" ]]; then
  echo "manifest not found: ${manifest_src}" >&2
  exit 2
fi

# Parse id, version, and the agent block's runtime + entrypoint (the in-archive
# binary path). The agent fields are read out of the `agent:` block specifically:
# a first-match grep over the whole manifest would pick up the `gcs:` block's
# entrypoint on any manifest that declares the gcs half first.
plugin_id="$(grep -E '^id:' "${manifest_src}" | head -n1 | sed -E 's/^id:[[:space:]]*//; s/[[:space:]]*$//')"
plugin_version="$(grep -E '^version:' "${manifest_src}" | head -n1 | sed -E 's/^version:[[:space:]]*"?([^"]*)"?/\1/')"
runtime="$(manifest_agent_runtime "${manifest_src}")"
entrypoint="$(manifest_agent_entrypoint "${manifest_src}")"

if [[ -z "${plugin_id}" || -z "${plugin_version}" ]]; then
  echo "could not parse plugin id/version from manifest" >&2
  exit 1
fi
if [[ "${runtime}" != "rust" ]]; then
  echo "manifest agent.runtime is '${runtime:-python}', not 'rust'; use scripts/pack.sh" >&2
  exit 1
fi
if [[ -z "${entrypoint}" ]]; then
  echo "manifest has no agent.entrypoint" >&2
  exit 1
fi

if [[ -z "${payload_list}" ]]; then
  # The crate name is the directory under the extension's agent/ folder. The
  # convention is one crate per extension agent half, named after the binary the
  # entrypoint references.
  crate="$(basename "${entrypoint}")"

  # Build the binary for the requested target.
  "${repo_root}/scripts/build-rust.sh" "${crate}" "${target}"
  bin_path="${repo_root}/target/${target}/release/${crate}"
  if [[ ! -f "${bin_path}" ]]; then
    echo "expected binary not found: ${bin_path}" >&2
    exit 1
  fi

  # Build the GCS bundle if this extension has a GCS half (esbuild ->
  # plugin.bundle.js). This packer used to rely on a stale, un-tracked local
  # bundle; it now builds the same way pack.sh does, so a clean CI checkout
  # ships a real bundle instead of a manifest pointing at a missing entrypoint.
  if [[ -d "${ext_dir}/gcs" ]]; then
    echo "building gcs bundle..."
    (cd "${ext_dir}/gcs" && pnpm build)
  fi
fi

archive_name="${plugin_id}-${plugin_version}.adosplug"
dist_dir="${ext_dir}/dist"
mkdir -p "${dist_dir}"
archive_path="${dist_dir}/${archive_name}"
rm -f "${archive_path}"

stage="$(mktemp -d)"
trap 'rm -rf "${stage}"' EXIT

if [[ -z "${payload_list}" ]]; then
  # Stage the manifest and assets. The shared exclude list plus 'agent' (the Rust
  # source tree; the compiled binary is staged separately) and 'target' (the
  # build output).
  rsync -a "${PACK_RSYNC_EXCLUDES[@]}" \
    --exclude 'target' \
    --exclude 'agent' \
    "${ext_dir}/" "${stage}/"

  # Place the compiled binary at the manifest entrypoint path inside the archive.
  mkdir -p "${stage}/$(dirname "${entrypoint}")"
  cp "${bin_path}" "${stage}/${entrypoint}"
  chmod 0755 "${stage}/${entrypoint}"

  # Never publish a half-archive: the agent binary and, when declared, the built
  # GCS bundle must both be present in the stage.
  assert_entrypoint_in_stage "${stage}" "${entrypoint}" "agent.entrypoint"
  assert_entrypoint_in_stage "${stage}" "$(manifest_gcs_entrypoint "${manifest_src}")" "gcs.entrypoint"
else
  # Payload paths are fetched at install and may not also ship in the archive
  # (the host refuses a payload that would overwrite an archive file), so each
  # one is excluded by its anchored path. The Rust crate sources never ship;
  # agent/python/ holds helper scripts the binaries run, so it does.
  payload_excludes=()
  while read -r payload_path; do
    payload_excludes+=(--exclude "/${payload_path}")
  done < <(awk '!/^[[:space:]]*(#|$)/ { print $2 }' "${payload_list}")

  rsync -a --prune-empty-dirs "${PACK_RSYNC_EXCLUDES[@]}" \
    ${payload_excludes[@]+"${payload_excludes[@]}"} \
    --exclude 'target' \
    --include '/agent/' \
    --include '/agent/python/***' \
    --exclude '/agent/***' \
    "${ext_dir}/" "${stage}/"

  assert_entrypoint_in_stage "${stage}" "$(manifest_gcs_entrypoint "${manifest_src}")" "gcs.entrypoint"

  python3 - "${stage}/manifest.yaml" "${payload_list}" "${payload_arch_os}" "${stage}" <<'PY'
import hashlib
import re
import sys
from pathlib import Path
from urllib.parse import urlsplit

manifest_path = Path(sys.argv[1])
list_path = Path(sys.argv[2])
arch_filter = [a for a in sys.argv[3].split(",") if a]
stage = Path(sys.argv[4])

# Mirrors of the host's payload rules, so a bad archive fails here instead of
# at install: ados-plugin-host src/manifest.rs (validate_payload,
# PAYLOAD_MAX_BYTES) and src/download.rs (HOST_SUFFIXES).
PAYLOAD_MAX_BYTES = 1024 * 1024 * 1024
HOST_SUFFIXES = (
    ".convex.cloud",
    ".convex.altnautica.com",
    "github.com",
    "objects.githubusercontent.com",
    "release-assets.githubusercontent.com",
    ".amazonaws.com",
    "localhost",
)
PROFILES = {"drone", "ground-station", "ground_station", "workstation", "compute"}
ARCH_OS = re.compile(r"^[a-z0-9_]+-[a-z0-9_]+$")


def fail(msg: str) -> None:
    raise SystemExit(f"FATAL: {msg}")


def host_allowed(host: str) -> bool:
    for suffix in HOST_SUFFIXES:
        if suffix.startswith("."):
            if host.endswith(suffix):
                return True
        elif host == suffix or (suffix != "localhost" and host.endswith("." + suffix)):
            return True
    return False


def relative_posix(path: str) -> bool:
    parts = path.split("/")
    return bool(path) and not path.startswith("/") and "\\" not in path and all(
        p not in ("", ".", "..") for p in parts
    )


# ---- the asset list -------------------------------------------------------
payloads = []
for lineno, raw in enumerate(list_path.read_text().splitlines(), 1):
    line = raw.strip()
    if not line or line.startswith("#"):
        continue
    cols = line.split()
    if len(cols) != 5:
        fail(f"{list_path}:{lineno}: expected 5 columns (name path arch_os profiles url), got {len(cols)}")
    name, path, arch_os, profiles, url = cols
    where = f"{list_path}:{lineno} ({path})"
    if not relative_posix(path):
        fail(f"{where}: payload path must be a relative posix path")
    if any(p["path"] == path for p in payloads):
        fail(f"{where}: duplicate payload path")
    parsed = urlsplit(url)
    if parsed.scheme != "https":
        fail(f"{where}: source {url!r} must be an https:// URL (the host refuses any other scheme)")
    if not parsed.hostname or not host_allowed(parsed.hostname):
        fail(f"{where}: source host {parsed.hostname!r} is not on the host's download allowlist")
    arch = None if arch_os == "-" else arch_os
    if arch is not None and not ARCH_OS.match(arch):
        fail(f"{where}: arch_os {arch!r} must be <arch>-<os>")
    profile_list = None if profiles == "-" else profiles.split(",")
    if profile_list is not None and (not profile_list or any(p not in PROFILES for p in profile_list)):
        fail(f"{where}: profiles {profiles!r} must be a comma-separated subset of {sorted(PROFILES)}")
    file = list_path.parent / name
    if not file.is_file():
        fail(f"{where}: asset {file} not found")
    digest = hashlib.sha256()
    size = 0
    with file.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
            size += len(chunk)
    if not 1 <= size <= PAYLOAD_MAX_BYTES:
        fail(f"{where}: size {size} is outside 1..={PAYLOAD_MAX_BYTES}")
    if (stage / path).exists():
        fail(f"{where}: the archive also ships this path; a payload may not overwrite an archive file")
    payloads.append(
        {"path": path, "source": url, "sha256": digest.hexdigest(), "size_bytes": size,
         "arch_os": arch, "profiles": profile_list}
    )
if not payloads:
    fail(f"{list_path} lists no payloads")

# ---- the manifest's agent block --------------------------------------------
lines = manifest_path.read_text().splitlines()


def indent(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def meaningful(line: str) -> bool:
    s = line.strip()
    return bool(s) and not s.startswith("#")


try:
    agent_start = next(i for i, l in enumerate(lines) if re.match(r"^agent:\s*(#.*)?$", l))
except StopIteration:
    fail("manifest has no agent: block")
agent_end = next(
    (i for i in range(agent_start + 1, len(lines)) if re.match(r"^[A-Za-z]", lines[i])),
    len(lines),
)
key_indent = next(indent(l) for l in lines[agent_start + 1 : agent_end] if meaningful(l))


def key_block(key: str):
    """(start, end) of an agent-level key's lines, end exclusive; None if absent."""
    pat = re.compile(rf"^ {{{key_indent}}}{key}:")
    for i in range(agent_start + 1, agent_end):
        if pat.match(lines[i]):
            end = i + 1
            while end < agent_end and (not lines[end].strip() or indent(lines[end]) > key_indent):
                end += 1
            # Trailing blank lines belong to the gap before the next key.
            while end > i + 1 and not lines[end - 1].strip():
                end -= 1
            return i, end
    return None


def strip_scalar(v: str) -> str:
    v = re.sub(r"\s+#.*$", "", v).strip()
    return v.strip('"').strip("'")


def flow_map(text: str) -> dict:
    body = text.strip()
    if not (body.startswith("{") and body.endswith("}")):
        fail(f"cannot read binaries entry {text!r}")
    out = {}
    for item in filter(None, (s.strip() for s in body[1:-1].split(","))):
        k, _, v = item.partition(":")
        out[strip_scalar(k)] = strip_scalar(v)
    return out


binaries: dict = {}
span = key_block("binaries")
if span is not None:
    start, end = span
    current = None
    child = None
    for line in lines[start + 1 : end]:
        if not meaningful(line):
            continue
        if child is None:
            child = indent(line)
        name, _, rest = line.strip().partition(":")
        name = strip_scalar(name)
        if indent(line) == child:
            rest = re.sub(r"\s+#.*$", "", rest).strip()
            binaries[name] = flow_map(rest) if rest else {}
            current = name
        elif current is not None:
            binaries[current][name] = strip_scalar(rest)

agent_text = "\n".join(lines[agent_start:agent_end])
refs = sorted(set(re.findall(r"bin:([A-Za-z0-9._-]+)", agent_text)))
for ref in refs:
    if ref not in binaries:
        fail(f"bin:{ref} is referenced but agent.binaries has no {ref!r} entry")

all_arch = sorted({a for per in binaries.values() for a in per})
covered_arch = arch_filter or all_arch
unknown = [a for a in covered_arch if a not in all_arch]
if unknown:
    fail(f"--arch-os {unknown} names no arch in agent.binaries ({all_arch})")
by_path = {p["path"]: p for p in payloads}
for name, per in sorted(binaries.items()):
    for arch, path in sorted(per.items()):
        payload = by_path.get(path)
        if payload is not None and payload["arch_os"] not in (None, arch):
            fail(f"binaries.{name}.{arch} is {path}, but its payload is arch_os {payload['arch_os']}")
        if arch in covered_arch and payload is None:
            fail(f"binaries.{name}.{arch} ({path}) has no payload entry")

# ---- rewrite the staged payloads block --------------------------------------
pad = " " * key_indent
out = [f"{pad}payloads:"]
for p in payloads:
    out.append(f'{pad}  - path: "{p["path"]}"')
    out.append(f'{pad}    source: "{p["source"]}"')
    out.append(f'{pad}    sha256: "{p["sha256"]}"')
    out.append(f'{pad}    size_bytes: {p["size_bytes"]}')
    if p["arch_os"]:
        out.append(f'{pad}    arch_os: {p["arch_os"]}')
    if p["profiles"]:
        out.append(f'{pad}    profiles: [{", ".join(p["profiles"])}]')
span = key_block("payloads")
if span is None:
    insert_at = agent_end
    while insert_at > agent_start + 1 and not lines[insert_at - 1].strip():
        insert_at -= 1
    lines[insert_at:insert_at] = out
else:
    lines[span[0] : span[1]] = out
manifest_path.write_text("\n".join(lines) + "\n")

print(f"staged {len(payloads)} payloads (binaries covered for: {', '.join(covered_arch)})")
for p in payloads:
    print(
        f"  {p['path']:<42} {p['arch_os'] or 'any':<14} "
        f"{','.join(p['profiles']) if p['profiles'] else 'all':<20} "
        f"{p['size_bytes']:>11}  {p['sha256'][:12]}  {p['source']}"
    )
PY
fi

(cd "${stage}" && zip -qr "${archive_path}" .)

echo "wrote ${archive_path}"
if [[ -z "${payload_list}" ]]; then
  echo "  runtime: rust, target: ${target}, entrypoint: ${entrypoint}"
else
  echo "  runtime: rust, payloads from: ${payload_list}, entrypoint: ${entrypoint}"
fi
echo "next: ADOS_SIGNING_KEY=<key> ${repo_root}/scripts/sign.sh ${archive_path}"
