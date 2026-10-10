#!/usr/bin/env bash
#
# Submit a signed .adosplug archive to a plugin registry.
#
# Usage:
#   ADOS_REGISTRY_TOKEN=<token> scripts/submit.sh \
#     dist/<plugin>.signed.adosplug \
#     --repo-url https://github.com/owner/repo \
#     --category drivers
#
# Optional environment variables:
#   ADOS_REGISTRY_URL   Base URL. Defaults to https://registry.ados.altnautica.com
#   ADOS_REGISTRY_TOKEN Bearer token tied to the publisher account.
#
# Categories (required): drivers, ui, ai, telemetry, tools.
#
# The registry form fields (plugin id, version, manifest, signature, signer
# id, payload hash, version bounds) are derived from the archive by
# scripts/registry_submit_fields.py, which needs python3 with PyYAML.
#
# Exit codes follow the agent's CLI envelope:
#   0  success
#   2  usage error
#  10  registry rejected the submission
#  30  network error
#
# This script does not pack or sign. Run scripts/pack.sh and
# scripts/sign.sh first; pass the resulting *.signed.adosplug here.

set -euo pipefail

usage() {
  cat <<EOF >&2
usage: submit.sh <archive.signed.adosplug> --category CAT [--repo-url URL]

Required env: ADOS_REGISTRY_TOKEN
Optional env: ADOS_REGISTRY_URL (default https://registry.ados.altnautica.com)
EOF
  exit 2
}

if [[ $# -lt 1 ]]; then
  usage
fi

archive="$1"
shift

repo_url=""
category=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-url)
      repo_url="$2"
      shift 2
      ;;
    --category)
      category="$2"
      shift 2
      ;;
    *)
      echo "unknown flag: $1" >&2
      usage
      ;;
  esac
done

if [[ ! -f "${archive}" ]]; then
  echo "archive not found: ${archive}" >&2
  exit 2
fi

if [[ -z "${ADOS_REGISTRY_TOKEN:-}" ]]; then
  echo "ADOS_REGISTRY_TOKEN is required" >&2
  exit 2
fi

if [[ -z "${category}" ]]; then
  echo "--category is required (drivers ui ai telemetry tools)" >&2
  exit 2
fi
case "${category}" in
  drivers|ui|ai|telemetry|tools) ;;
  *)
    echo "invalid category '${category}'; expected one of: drivers ui ai telemetry tools" >&2
    exit 2
    ;;
esac

if ! python3 -c 'import yaml' 2>/dev/null; then
  echo "python3 with PyYAML is required (pip install pyyaml)" >&2
  exit 2
fi

base_url="${ADOS_REGISTRY_URL:-https://registry.ados.altnautica.com}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

response="$(mktemp)"
manifest_file="$(mktemp)"
trap 'rm -f "${response}" "${manifest_file}"' EXIT

fields="$(python3 "${script_dir}/registry_submit_fields.py" "${archive}" "${manifest_file}")" || exit 2

form_args=(
  -F "archive=@${archive}"
  -F "manifest_yaml=<${manifest_file}"
  --form-string "category=${category}"
)
if [[ -n "${repo_url}" ]]; then
  form_args+=( --form-string "repo_url=${repo_url}" )
fi
while IFS= read -r line; do
  [[ -z "${line}" ]] && continue
  form_args+=( --form-string "${line}" )
done <<< "${fields}"

http_status="$(curl --silent --show-error \
  --output "${response}" \
  --write-out "%{http_code}" \
  --max-time 60 \
  --request POST \
  --header "Authorization: Bearer ${ADOS_REGISTRY_TOKEN}" \
  "${form_args[@]}" \
  "${base_url}/v1/plugins/submit")" || {
    echo "network error talking to ${base_url}" >&2
    exit 30
  }

if [[ "${http_status}" -ge 200 && "${http_status}" -lt 300 ]]; then
  cat "${response}"
  echo
  exit 0
fi

echo "registry rejected submission (HTTP ${http_status}):" >&2
cat "${response}" >&2
echo >&2
exit 10
