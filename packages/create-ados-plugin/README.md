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

- `manifest.yaml` with the template's permissions and contributes block
- A skeleton GCS bundle (`gcs/src/plugin.ts`) that mounts in the FC tab
- A skeleton agent subprocess (`agent/plugin.py`) for hybrid/agent-only
- An `en.json` locale bundle
- A README pointing at `docs.altnautica.com/developers/`

## License

GPL-3.0-or-later.
