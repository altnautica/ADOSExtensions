# Rust extension agent halves

An extension's agent half can be written in Rust instead of Python. A Rust
agent half ships as its own compiled binary that the agent plugin host starts
directly under systemd. It talks to the host over the same wire a Python agent
half uses: length-prefixed msgpack frames over a per-plugin Unix socket, gated
by a capability token.

This note covers how a Rust agent half is laid out, built, and packed. The
worked minimal example is `extensions/_hello-rust`.

## Layout

```
extensions/<name>/
  manifest.yaml            # agent.runtime: rust, agent.entrypoint: bin/<name>
  agent/
    Cargo.toml             # the crate; a workspace member
    src/main.rs            # impl Plugin + run_plugin
  locales/ ...             # optional assets, same as any extension
```

The Rust crates form a third workspace alongside the existing pnpm and uv
workspaces. The Cargo workspace lives at the repository root (`Cargo.toml`) and
lists each Rust agent crate explicitly under `members`. To add a Rust
extension, create `extensions/<name>/agent/Cargo.toml` and add
`"extensions/<name>/agent"` to the workspace `members`.

## The SDK dependency

The crate depends on `ados-sdk`, the plugin author SDK. The SDK provides the
IPC client, the plugin-facing context facade, the hardware driver traits, and
the lifecycle runner. It pulls in `ados-protocol` (the frame, envelope, and
capability-token wire) transitively, so a crate that names only `ados-sdk` also
gets the protocol types, including the vision framebus contract.

For local development the dependency is a path to the sibling checkout, set
once in the root `Cargo.toml`:

```toml
[workspace.dependencies]
ados-sdk = { path = "../ADOSDroneAgent/crates/ados-sdk" }
```

This assumes the two repositories are checked out side by side:

```
<root>/ADOSDroneAgent
<root>/ADOSExtensions
```

A member crate then inherits it with `ados-sdk = { workspace = true }`.

In CI, where the sibling checkout is not present, override the dependency with
a git source pinned to a commit:

```toml
ados-sdk = { git = "https://github.com/altnautica/ADOSDroneAgent", package = "ados-sdk", rev = "<commit>" }
```

Pin a `rev` rather than a branch so a plugin build is reproducible and does not
shift when the agent repository moves.

## Writing the plugin

The binary implements the `Plugin` lifecycle trait and calls `run_plugin` from
`main`. Every lifecycle hook has a default no-op, so a plugin overrides only the
hooks it needs. The runner reads `--socket` (with the `ADOS_PLUGIN_SOCKET` env
fallback) plus the capability token and agent id, connects, builds a
`PluginContext`, and drives `on_install` through `on_disable` until the unit
stops.

The empty example in `extensions/_hello-rust/agent/src/main.rs` is the smallest
form: an empty `Plugin`, a SIGTERM/SIGINT shutdown future, and the `run_plugin`
call. A real plugin uses the `PluginContext` sub-clients (`events`, `mavlink`,
`telemetry`, `camera`, and so on) inside its hooks, and lists the matching
capabilities under `agent.permissions` in the manifest.

## Manifest

A Rust agent half sets two fields on the `agent` block:

```yaml
agent:
  runtime: rust
  entrypoint: "bin/<name>"   # path the binary takes inside the archive
```

`runtime: rust` tells the host to exec the binary directly instead of running
the shared Python runner. `entrypoint` is the path of the binary inside the
installed plugin tree. The host execs
`<install_dir>/<id>/<entrypoint> --socket <per-plugin-socket>` and delivers the
capability token and agent id through the unit environment, so the binary never
sees secrets on its command line.

`agent.permissions` lists the capabilities the binary calls. An empty list is
valid for a plugin that makes no IPC calls. A vision plugin, for example, lists
`vision.frame.read`, `vision.model.register`, and `vision.detection.publish`.

## Build

Drone-class ADOS boards are aarch64 Linux. Build the binary for the static musl
target so it carries no libc dependency:

```bash
scripts/build-rust.sh <crate-name>
# equivalent to:
cargo build --release --target aarch64-unknown-linux-musl -p <crate-name>
```

A static aarch64 build needs an aarch64 musl linker. On a host that is not
aarch64 Linux, supply one with the cargo linker env var:

```bash
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=aarch64-linux-musl-gcc \
  scripts/build-rust.sh <crate-name>
```

or run the build inside a CI runner / container that has the aarch64 musl cross
toolchain installed. The compile step is host-independent; only the final link
needs the cross linker. A host build (`cargo build -p <crate-name>`, no
`--target`) is enough to check that the code compiles and links against the
SDK during development.

The release profile in the root workspace strips the binary and applies LTO, so
the packed binary stays small.

## Pack

Pack the built binary and the manifest into an unsigned `.adosplug` archive:

```bash
scripts/pack-rust.sh <name>            # e.g. _hello-rust
scripts/pack-rust.sh <name> <target>   # override the target triple
```

`pack-rust.sh` reads `agent.entrypoint` from the manifest, builds the crate for
the target, and stages the manifest plus the binary at the entrypoint path.
The archive layout is:

```
manifest.yaml
bin/<name>          # the compiled aarch64 binary at agent.entrypoint
<assets>            # locales, config-schema.json, README, gcs bundle if present
```

The Rust source tree (`agent/`), build output (`target/`, `dist/`), and test
scaffolding are excluded from the archive.

## Sign

Signing is the final step and is the same for every runtime. The signature
covers every archive entry, so the binary is protected by the same signature as
the manifest:

```bash
ADOS_SIGNING_KEY=/path/to/key.ed25519 scripts/sign.sh dist/<plugin-id>-<version>.adosplug
```

`sign.sh` computes the canonical payload hash (sha256 over the sorted
`<path>\n<sha256-hex>\n` lines of every entry except `SIGNATURE`), signs the
32-byte digest with the publisher Ed25519 key, and writes a `SIGNATURE` file
(signer key id, then base64 signature) into a `*.signed.adosplug` archive. The
agent verifies this signature against the public keys it ships before
installing. The private key is never required to build or pack; it is held by
the release pipeline.
