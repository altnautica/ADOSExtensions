# create-ados-plugin

Scaffold a new ADOS plugin from a template. Three templates:
`gcs-only`, `agent-only`, `hybrid`.

## Usage

```sh
npx create-ados-plugin my-plugin
# or
npx create-ados-plugin my-plugin --half hybrid --id com.example.my-plugin --author "You"
```

The CLI prompts for missing values. With `--half`, `--id`, `--author`,
or piped input, it runs unattended.

## What you get

- `manifest.yaml` with the template's permissions and `contributes` block
- A skeleton GCS bundle (`gcs/src/plugin.ts`) and unit tests
- A skeleton agent subprocess (`agent/plugin.py`) and pytest tests, for
  hybrid/agent-only
- An `en.json` locale bundle
- A README pointing at `docs.altnautica.com/developers/`

The **hybrid** template is the living "everything a plugin can do"
example. It demonstrates the full platform from one manifest:

- a flight **Skill** in the cockpit Skill Bar,
- a **node-detail tab** (the `node.detail.tab` slot, narrowed to the
  `drone` profile),
- native declarative **settings** (`contributes.parameters`) — a range
  slider, an enum dropdown, and a **model picker** bound to
  `engine.detector`,
- and a **model requirement** (`contributes.models`) with per-board
  variants.

The `gcs-only` and `agent-only` templates stay minimal.

## License

GPL-3.0-or-later.
