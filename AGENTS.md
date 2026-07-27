# AGENTS.md - ADOSExtensions

Agentic coding instructions for ADOS plugin extensions.

## Purpose

Work in this repository as an engineering agent for first-party ADOS
extensions. Each extension can include a GCS half, an agent half, manifests,
locales, tests, packaging output, and its own changelog. Keep extension
contracts explicit and permission-scoped.

## Read First

- Check `git status --short` before edits and preserve unrelated changes.
- Identify the extension being touched before changing shared workspace files.
- Inspect that extension's manifest, source, tests, build output convention, and
  changelog before editing.
- Keep manifests accurate. Permissions must match the runtime surface the
  extension actually uses.
- Build and test only the changed extension unless shared tooling changed.

## Stack and Commands

- pnpm workspaces for TypeScript and React GCS halves.
- Python 3.11 workspace support for agent halves.
- Each extension owns its manifest, source, tests, build output, and changelog.
- Common commands:

```bash
pnpm install
pnpm build:battery-health
pnpm test:battery-health
pnpm build:thermal-camera
pnpm test:thermal-camera
pnpm build:mavlink-gimbal-v2
pnpm test:mavlink-gimbal-v2
```

For Python agent halves, run focused tests from the changed agent package using
the local environment or workspace tooling already present in that extension.

## Repository Map

- Battery health panel:
  `extensions/battery-health-panel/`
- Thermal camera extension:
  `extensions/thermal-camera-flir-lepton-usb/`
- MAVLink gimbal extension:
  `extensions/mavlink-gimbal-v2/`
- Follow-me extension:
  `extensions/follow-me/`
- SIYI optical pod extension:
  `extensions/siyi-pod/`
- Vision-nav extension:
  `extensions/vision-nav/`
- GCS halves: `extensions/*/gcs/`
- Agent halves: `extensions/*/agent/`
- Locales: `extensions/*/locales/`
- Packaged output: `extensions/*/dist/`

Do not commit generated archives unless the release workflow explicitly expects
them.

## Extension Rules

- GCS halves should keep UI isolated to declared slots and avoid coupling to
  host app internals.
- Agent halves should validate inputs, enforce permissions, and fail closed
  when hardware or services are unavailable.
- Keep extension manifests, permissions, capabilities, version, changelog, and
  docs aligned with actual behavior.
- Treat the manifest as the public contract. Do not add runtime behavior that is
  not represented there.
- Keep each extension independently versioned and documented.
- Keep hardware-aware code behind boundaries that can be tested without the
  device attached.
- Keep locale keys stable and update locale files when user-visible extension UI
  changes.

## Working in the Open

This is a public, open-source repository. Every commit, diff, and branch is
visible the moment it is pushed and stays in history permanently, so a mistake
cannot be un-published by deleting it later. Review what a change actually
contains before committing.

- **Never commit secrets.** API keys, tokens, deploy keys, passwords, private
  certificates, and `.env` files stay out of the tree. Generated secrets belong
  only in gitignored files. If a secret does land in a commit, treat it as
  compromised and rotate it.
- **Never commit real deployment detail.** Hostnames, IP addresses, tunnel
  names, device identifiers, and account names from a live setup are an attack
  surface. Use placeholders such as `example-oem`, `cloud.example.com`,
  `192.168.1.50`, and `mycompany-fleet`.
- **Never commit other people's data.** Personal names, email addresses,
  customer or employer names, real flight logs and GPS traces, and raw log
  dumps that contain any of the above do not belong in a public repository.
- **Tests and manifests are published too.** Fixtures, sample payloads, locale
  files, and packaging metadata get the same care as source.
- **Respect licensing when bringing in outside code.** Third-party source is
  vendored into a vendor directory with its license intact and is never pasted
  into our own modules.
- **Keep contributions technical.** Architecture, APIs, commands, schemas,
  configuration, hardware interfaces, deployment, and troubleshooting.
  Commercial, pricing, or roadmap commentary does not belong in the codebase.
- **Comments, log strings, commit messages, and PR titles are public too.** Keep
  them bland, factual, and technical.

## Verification

- GCS half change: run that extension's `pnpm build:*` and `pnpm test:*`
  command.
- Agent half change: run focused Python tests for the changed agent package and
  any lint/type checks already configured there.
- Manifest or permission change: verify the manifest matches actual runtime
  usage and that permission enforcement fails closed.
- Locale or UI text change: verify locale files and rendered labels for the
  changed extension.
- Shared workspace change: run every affected extension build/test script.

Before finalizing, run `git diff --check` and report any skipped checks.

## Review Expectations

When reviewing, list findings first and focus on permission mismatches, host
contract coupling, unvalidated inputs, hardware fallback gaps, stale manifests,
missing tests, broken packaging, and locale drift. Cite file and line
references.

For implementation work, keep changes within the touched extension unless shared
tooling or host contracts require a broader update.

## Cross-Repo Impact

- Mission Control host changes may require GCS half updates.
- Drone Agent plugin host or permission changes may require agent half and
  manifest updates.
- Extension authoring, packaging, or operator workflow changes may require
  Documentation updates.

## Related Public Projects

- [ADOS Drone Agent](https://github.com/altnautica/ADOSDroneAgent) - Python
  agent runtime for extension agent halves.
- [ADOS Mission Control](https://github.com/altnautica/ADOSMissionControl) -
  browser GCS runtime for extension UI halves.
- [ADOS Documentation](https://github.com/altnautica/Documentation) - public
  docs for plugin authoring and distribution.
