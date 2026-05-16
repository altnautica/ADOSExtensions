/**
 * Public surface of @altnautica/extension-ui.
 *
 * Consumers import directly from the package root:
 *
 *   import { Wizard, CameraPreview, PrimaryButton } from "@altnautica/extension-ui";
 *
 * Internal modules can deep-import specific files when they need to
 * avoid a full barrel pull. The README documents the public set.
 */

// Wizard
export { Wizard } from "./wizard/Wizard";
export { WizardStep } from "./wizard/WizardStep";
export { StepIndicator } from "./wizard/StepIndicator";
export { useWizardState } from "./wizard/useWizardState";
export type {
  WizardState,
  WizardStateOptions,
} from "./wizard/useWizardState";

// Media
export { CameraPreview } from "./media/CameraPreview";
export type {
  DetectionCorner,
  DetectionShape,
} from "./media/CameraPreview";
export { FramePreview } from "./media/FramePreview";
export type { CapturedFrame } from "./media/FramePreview";
export { PoseCoverageMap } from "./media/PoseCoverageMap";
export type { PoseSample } from "./media/PoseCoverageMap";

// Controls
export { FilePicker } from "./controls/FilePicker";
export { PrimaryButton } from "./controls/PrimaryButton";
export { SecondaryButton } from "./controls/SecondaryButton";
export { ProgressBar } from "./controls/ProgressBar";
export { ResultBanner } from "./controls/ResultBanner";

// Feedback
export { Toast } from "./feedback/Toast";
export { DiagnosticTable } from "./feedback/DiagnosticTable";
export type { DiagnosticRow } from "./feedback/DiagnosticTable";

// Theme
export { TOKENS } from "./theme/tokens";
export type { ThemeToken } from "./theme/tokens";
