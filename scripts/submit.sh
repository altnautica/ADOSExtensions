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
# Categories: drivers, ui, ai, telemetry, tools.
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
usage: submit.sh <archive.signed.adosplug> [--repo-url URL] [--category CAT]

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

base_url="${ADOS_REGISTRY_URL:-https://registry.ados.altnautica.com}"

form_args=( -F "archive=@${archive}" )
if [[ -n "${repo_url}" ]]; then
  form_args+=( -F "repo_url=${repo_url}" )
fi
if [[ -n "${category}" ]]; then
  form_args+=( -F "category=${category}" )
fi

response="$(mktemp)"
trap 'rm -f "${response}"' EXIT

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
