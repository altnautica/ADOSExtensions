# GitHub Actions workflows

Workflows that build, test, sign, and publish first-party plugin
extensions. Each workflow targets a different tag namespace so a
release manager can pick the pipeline that matches the situation.

## Tag namespaces

| Workflow                       | Trigger tag pattern                | What it does |
|--------------------------------|------------------------------------|-------------|
| `release.yml`                  | `battery-health-panel-v*`, `thermal-camera-flir-lepton-usb-v*`, `mavlink-gimbal-v2-v*` | Per-extension legacy release: builds, packs with `scripts/pack.sh`, signs with `scripts/sign.sh`, publishes. |
| `vision-nav-release.yml`       | `vision-nav-v*`                    | Specialised pipeline for vision-nav, builds vendor binaries in a matrix job then packs and signs. |
| `vision-nav-vendor-binaries.yml` | `workflow_dispatch` (manual)     | Standalone vendor-binary rebuild for vision-nav. |
| `sign-release.yml`             | `extensions/<ext>-v*`              | Generic pack-and-sign pipeline using the `ados plugin sign` CLI shipped by the drone agent. Targets any extension under `extensions/`. |

## Triggering a release with the generic sign pipeline

The `sign-release.yml` workflow uses the canonical signing CLI from
the drone agent, so the bytes it produces are guaranteed to match
what the agent verifies at install time. Use it for new extensions
that do not yet have a custom pipeline.

```bash
# After landing the extension code on main:
git tag extensions/my-new-extension-v0.1.0
git push origin extensions/my-new-extension-v0.1.0
```

The tag triggers the workflow. The job:

1. Checks out the repo with submodules.
2. Installs the drone agent from git so the `ados plugin sign`
   CLI is on `PATH`.
3. Loads the signing key from the `ALTNAUTICA_PLUGIN_KEY_A`
   repository secret. The secret is the base64-encoded contents of
   the Ed25519 private PEM. See
   `ADOSDroneAgent/docs/plugin-signing/key-generation.md` for the
   founder-side runbook.
4. Runs `ados plugin sign extensions/<name> --key … --signer-id
   altnautica-2026-A --output dist/com.altnautica.<name>-<ver>.signed.adosplug`.
5. Wipes the private key from the runner.
6. Uploads the signed `.adosplug` and its `.sha256` sidecar to the
   GitHub Release matching the tag.

## Required secrets

| Secret name                  | Format                                   | Used by |
|------------------------------|------------------------------------------|--------|
| `ALTNAUTICA_PLUGIN_KEY_A`    | Base64-encoded Ed25519 private PEM (PKCS#8) | `sign-release.yml` |
| `ADOS_SIGNING_KEY`           | Base64-encoded Ed25519 private PEM       | `release.yml`, `vision-nav-release.yml` (legacy `scripts/sign.sh` path) |
| `ADOS_SIGNING_KEY_ID`        | Signer id string (e.g. `altnautica-2026-A`) | `release.yml`, `vision-nav-release.yml` |

Set secrets via GitHub repo settings: **Settings > Secrets and
variables > Actions**.

## Dry-running a workflow locally

The `sign-release.yml` workflow steps are reproducible on a developer
workstation:

```bash
# Install the agent so `ados plugin sign` is on PATH.
pip install "git+https://github.com/altnautica/ADOSDroneAgent.git"

# Mint a throwaway keypair if you do not have one.
ados plugin keygen test-signer --output-dir /tmp/test-keys

# Sign an extension as the workflow would.
ados plugin sign extensions/vision-nav \
    --key /tmp/test-keys/test-signer.priv.pem \
    --signer-id test-signer \
    --output /tmp/vision-nav.signed.adosplug

# Walk the result through the agent's normal install path on a paired rig.
```

Throwaway signer ids are accepted at install time only when the
matching public PEM lives in `/etc/ados/plugin-keys/`. To exercise
the production signature path with a throwaway key, drop the public
PEM into `/etc/ados/plugin-keys/` on the test rig manually before
running `ados plugin install`.
