import type { VisionNavTelemetry } from "../types";

/**
 * One step in the guided tour.
 *
 * ``targetTestId`` matches a ``data-testid`` on a card or button in
 * the Navigation tab; the orchestrator scrolls the target into view
 * and highlights it. ``title`` and ``body`` carry the step copy
 * (English defaults; the orchestrator resolves the matching i18n key
 * if a translation is registered). ``precondition`` optionally skips
 * the step when the telemetry does not support it.
 */
export interface TourStep {
  id: string;
  targetTestId: string;
  titleKey: string;
  titleFallback: string;
  bodyKey: string;
  bodyFallback: string;
  precondition?: (telemetry: VisionNavTelemetry) => boolean;
}

/**
 * The five-step first-run tour. The order matches how an operator
 * works through the tab on a new drone.
 */
export const TOUR_STEPS: TourStep[] = [
  {
    id: "mode-card",
    targetTestId: "vn-mode-card",
    titleKey: "navigation.tour.modeTitle",
    titleFallback: "Navigation mode",
    bodyKey: "navigation.tour.modeBody",
    bodyFallback:
      "The modes the agent can run, with the active one marked. Set " +
      "the mode in the plugin's per-drone settings.",
  },
  {
    id: "sensors-card",
    targetTestId: "vn-sensors-card",
    titleKey: "navigation.tour.sensorsTitle",
    titleFallback: "Sensor health",
    bodyKey: "navigation.tour.sensorsBody",
    bodyFallback:
      "Camera, IMU, and rangefinder health at a glance. The camera " +
      "row shows whether the agent loaded a camera calibration " +
      "(camchain.yaml in the plugin's data directory).",
  },
  {
    id: "estimator-card",
    targetTestId: "vn-estimator-card",
    titleKey: "navigation.tour.estimatorTitle",
    titleFallback: "Estimator state",
    bodyKey: "navigation.tour.estimatorBody",
    bodyFallback:
      "Live state of the active estimator. The pill shows init / " +
      "converging / converged / degraded / failed.",
  },
  {
    id: "telemetry-charts",
    targetTestId: "vn-telemetry-charts",
    titleKey: "navigation.tour.chartsTitle",
    titleFallback: "Telemetry trends",
    bodyKey: "navigation.tour.chartsBody",
    bodyFallback:
      "Sixty seconds of rolling history. Watch the flow quality stay " +
      "above the gate and the camera-IMU sync offset stay green.",
  },
  {
    id: "pre-arm",
    targetTestId: "vn-pre-arm",
    titleKey: "navigation.tour.preArmTitle",
    titleFallback: "Pre-arm checklist",
    bodyKey: "navigation.tour.preArmBody",
    bodyFallback:
      "Mode-aware arm-readiness. Every check must be green before " +
      "the drone is armable in this mode.",
  },
];

/**
 * localStorage key the orchestrator uses to record that the tour has
 * been completed at least once. A single key per browser session; we
 * do not have a per-drone identifier on the plugin context so the
 * tour persists across drones for now.
 */
export const TOUR_PERSIST_KEY = "vision-nav-tour-seen";
