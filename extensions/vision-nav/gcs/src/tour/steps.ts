import type { VisionNavTelemetry } from "../types";

/**
 * One step in the guided tour.
 *
 * ``targetTestId`` matches a ``data-testid`` on a card or button in
 * the Navigation tab; the orchestrator scrolls the target into view
 * and highlights it. ``title`` and ``body`` carry the step copy
 * (English defaults; the orchestrator resolves the matching i18n key
 * if a translation is registered). ``precondition`` optionally skips
 * the step when the gate fails (e.g. the VIO step is hidden on
 * agents that do not advertise VIO support).
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
 * The seven-step first-run tour. The order matches how an operator
 * works through the tab on a new drone.
 */
export const TOUR_STEPS: TourStep[] = [
  {
    id: "mode-card",
    targetTestId: "vn-mode-card",
    titleKey: "navigation.tour.modeTitle",
    titleFallback: "Pick a mode",
    bodyKey: "navigation.tour.modeBody",
    bodyFallback:
      "This is where you pick the estimator. Six modes; the card " +
      "filters the list against what the agent can actually run.",
  },
  {
    id: "sensors-card",
    targetTestId: "vn-sensors-card",
    titleKey: "navigation.tour.sensorsTitle",
    titleFallback: "Sensor health",
    bodyKey: "navigation.tour.sensorsBody",
    bodyFallback:
      "Camera, IMU, and rangefinder health at a glance. The " +
      "Calibrate button on the camera row uploads a Kalibr-style " +
      "camera-IMU calibration file.",
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
      "Sixty seconds of rolling history. Watch the sync offset " +
      "stay green and the feature count stay above twenty for " +
      "healthy VIO.",
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
  {
    id: "ekf-switcher",
    targetTestId: "vn-ekf-switcher",
    titleKey: "navigation.tour.ekfTitle",
    titleFallback: "EKF source switch",
    bodyKey: "navigation.tour.ekfBody",
    bodyFallback:
      "Runtime EKF source switch on ArduPilot. The escape hatch " +
      "back to GPS if vision goes degraded mid-flight.",
  },
  {
    id: "fallback-preview",
    targetTestId: "vn-tour-fallback-preview",
    titleKey: "navigation.tour.fallbackTitle",
    titleFallback: "Fallback banner",
    bodyKey: "navigation.tour.fallbackBody",
    bodyFallback:
      "If anything goes wrong, a banner appears here with a " +
      "suggested next action. This step shows you what it looks " +
      "like.",
  },
];

/**
 * localStorage key the orchestrator uses to record that the tour has
 * been completed at least once. A single key per browser session; we
 * do not have a per-drone identifier on the plugin context so the
 * tour persists across drones for now.
 */
export const TOUR_PERSIST_KEY = "vision-nav-tour-seen";
