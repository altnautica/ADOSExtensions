# AGENTS.md - ADOSExtensions

Agentic coding instructions for ADOS plugin extensions.

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

Build and package one extension from its extension folder, then use the
repository packaging scripts.

## Extension Guidelines

- Keep manifests accurate. Permissions must match the runtime surface the
  extension actually uses.
- GCS halves should keep UI isolated to declared slots and avoid coupling to
  host app internals.
- Agent halves should validate inputs, enforce permissions, and fail closed
  when hardware or services are unavailable.
- Keep each extension independently versioned and documented.
- Do not commit generated archives unless release workflow explicitly expects
  them.

## Repository Boundary

Keep repo instructions, docs, comments, tests, manifests, and examples
self-contained and technical. Document behavior through extension APIs,
commands, permissions, packaging, runtime behavior, and operator workflows.
Keep this repository self-contained. Describe integrations through documented
APIs, package names, public protocols, and public project links.

## Related Public Projects

- [ADOS Drone Agent](https://github.com/altnautica/ADOSDroneAgent) - Python
  agent runtime for extension agent halves.
- [ADOS Mission Control](https://github.com/altnautica/ADOSMissionControl) -
  browser GCS runtime for extension UI halves.
- [ADOS Documentation](https://github.com/altnautica/Documentation) - public
  docs for plugin authoring and distribution.
