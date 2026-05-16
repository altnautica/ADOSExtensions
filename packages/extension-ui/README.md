# @altnautica/extension-ui

Reusable React UI primitives for ADOS first-party extensions. The
package is internal-only (workspace symlink, not published) until the
API stabilises.

## What's in here

| Group | Components |
|---|---|
| `wizard/` | `Wizard`, `WizardStep`, `StepIndicator`, `useWizardState` |
| `media/` | `CameraPreview`, `FramePreview`, `PoseCoverageMap` |
| `controls/` | `FilePicker`, `PrimaryButton`, `SecondaryButton`, `ProgressBar`, `ResultBanner` |
| `feedback/` | `Toast`, `DiagnosticTable` |
| `theme/` | `tokens.ts` — shared CSS variable contract |

## Consumer recipe

Add to the extension's `gcs/package.json`:

```json
"dependencies": {
  "@altnautica/extension-ui": "workspace:*"
}
```

Then import:

```tsx
import { Wizard, CameraPreview, PrimaryButton } from "@altnautica/extension-ui";
```

The package ships TypeScript source directly (no build step) so
extensions consume via the bundler-level transpile they already run
for their own code.

## Theme contract

Every primitive reads from a documented set of CSS variables. The
consumer extension defines them at any wrapper level; the primitive
inherits. Default values are sensible so the primitives render
acceptably without any theme overrides.

| Variable | Default | Used by |
|---|---|---|
| `--ext-ui-accent` | `#2563eb` | PrimaryButton, focus rings |
| `--ext-ui-surface` | `rgba(255,255,255,0.02)` | card backgrounds |
| `--ext-ui-surface-2` | `rgba(255,255,255,0.06)` | hover states |
| `--ext-ui-border` | `rgba(255,255,255,0.08)` | dividers |
| `--ext-ui-text` | `#e5e7eb` | body copy |
| `--ext-ui-text-muted` | `#94a3b8` | secondary copy |
| `--ext-ui-ok` | `#34d399` | success indicators |
| `--ext-ui-warn` | `#f59e0b` | warning indicators |
| `--ext-ui-error` | `#ef4444` | error indicators |

## Design contract (for component authors)

Every primitive in this package obeys:

- Pure render. No side effects in the component body. State lives in
  hooks the consumer holds.
- Theme via CSS variables. No hardcoded colours, no Tailwind, no
  styled-components. Inline styles only, reading from the variables
  above.
- Accessible by default. ARIA roles, keyboard navigation, focus
  management built in.
- TypeScript strict. No `any`. All public props fully typed.
- Stable `data-testid` on every interactive element. Namespaced
  `ext-ui-<component>-<role>` so consumer tests stay terse.
- i18n-friendly. No hardcoded English. Strings come in via props; the
  consumer pipes its own `tr(t, key, fallback)` results.

## Tests

```sh
pnpm --filter @altnautica/extension-ui test
pnpm --filter @altnautica/extension-ui typecheck
```

Each primitive has at least one unit test under
`tests/<component>.test.tsx`. The Wizard orchestrator has a fuller
suite covering step navigation, dismiss, and replay.
