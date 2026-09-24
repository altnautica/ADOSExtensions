# GitHub Actions workflows

Workflows that build, test, sign, and publish first-party plugin
extensions. Each workflow targets a different tag namespace so a
release manager can pick the pipeline that matches the situation.

## Tag namespaces

| Workflow                       | Trigger tag pattern                | What it does |
|--------------------------------|------------------------------------|-------------|
| `release.yml`                  | `mavlink-gimbal-v2-v*`, `follow-me-v*`, `siyi-pod-v*` | Per-extension release: builds, packs with `scripts/pack.sh`, signs with `scripts/sign.sh`, publishes. The tag list is the publish allowlist. |
| `vision-nav-release.yml`       | `vision-nav-v*`                    | Pipeline for vision-nav: cross-compiles the Rust agent half against the SDK revision pinned in `ADOS_AGENT_REV`, then packs with `scripts/pack-rust.sh` and signs. |
| `world-engine-release.yml`     | `world-engine-v*`                  | Pipeline for the World Engine: builds its binaries per arch (aarch64-linux, x86_64-linux, aarch64-macos), attaches them and the Rerun viewer wasm to the release as install-time payloads, then packs the small archive with `scripts/pack-rust.sh --payload-manifest` and signs. |
| `rust-check.yml`               | pull request, push to main         | Builds, clippies and tests the Rust agent half against the pinned SDK revision. |
| `typecheck.yml`                | pull request, push to main         | TypeScript typecheck across every GCS package. |

## Publishing a new extension

An extension is publishable only once a tag pattern for it exists in
`release.yml`, or in the trigger of its own release workflow for the
extensions that have one (`vision-nav-v*`, `world-engine-v*`; never both,
or one tag publishes twice). Adding that pattern is the act that makes its archive
publishable, so do not add one for an extension whose advertised
capability cannot execute:
`thermal-camera-flir-lepton-usb` is deliberately absent because its
capture path needs a libuvc backend that is not in this repository.

```bash
# After adding the tag pattern and landing the extension code on main:
git tag my-new-extension-v0.1.0
git push origin my-new-extension-v0.1.0
```

The tag triggers `release.yml`. The job checks out the repo, installs
the workspace, runs that extension's GCS tests, packs the archive with
`scripts/pack.sh` (which refuses to write a half-archive: both the GCS
bundle and the module named by a Python `agent.entrypoint` have to be in
the stage), signs it with `scripts/sign.sh` against the
`ADOS_SIGNING_KEY` secret, and uploads the signed `.adosplug` to the
GitHub Release matching the tag. See
`ADOSDroneAgent/docs/plugin-signing/key-generation.md` for the
maintainer key-generation runbook.

## Required secrets

One repository secret is used. `ADOS_SIGNING_KEY`, `ADOS_SIGNING_KEY_INLINE`
and `ADOS_SIGNING_KEY_ID` are step environment variables the workflows set from
it, not secrets to configure.

| Secret name                  | Format                                   | Used by |
|------------------------------|------------------------------------------|--------|
| `ALTNAUTICA_PLUGIN_KEY_A`    | Base64-encoded Ed25519 private PEM (PKCS#8) | `release.yml`, `vision-nav-release.yml`, `world-engine-release.yml` |

Set secrets via GitHub repo settings: **Settings > Secrets and
variables > Actions**.

## World Engine payloads

The World Engine archive carries no binaries. Its manifest lists each binary
and the Rerun viewer wasm under `agent.payloads`, and a node fetches only the
entries for its own arch and profile at install. The asset table lives in
`scripts/world-engine-payloads.tsv`. To pack the same way on an Apple Silicon
Mac for bench testing (aarch64-macos payloads only):

```bash
ADOS_SIGNING_KEY=/path/to/bench-key.pem \
  scripts/pack-world-engine-local.sh /tmp/we-bench https://<allowlisted-host>/<path>
```

Serve the output directory at that base URL. The host accepts only an
`https://` payload source on its download allowlist, with a certificate that
chains to a public root.

## Reproducing the signing step locally

The signing step is reproducible on a developer workstation with the
agent's own CLI:

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
