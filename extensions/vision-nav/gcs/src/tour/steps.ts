import type { VisionNavTelemetry } from "../types";

/**
 * One step in the guided tour.
 *
 * ``targetTestId`` matches a ``data-testid`` on a card or button in
 * the Navigation tab; the orchestrator scrolls the target into view.
 * ``titleKey`` and ``bodyKey`` name the step copy in ``locales/en.json``.
 * ``precondition`` optionally skips the step when the telemetry does not
 * support it.
 */
export interface TourStep {
  id: string;
  targetTestId: string;
  titleKey: string;
  bodyKey: string;
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
    bodyKey: "navigation.tour.modeBody",
  },
  {
    id: "sensors-card",
    targetTestId: "vn-sensors-card",
    titleKey: "navigation.tour.sensorsTitle",
    bodyKey: "navigation.tour.sensorsBody",
  },
  {
    id: "estimator-card",
    targetTestId: "vn-estimator-card",
    titleKey: "navigation.tour.estimatorTitle",
    bodyKey: "navigation.tour.estimatorBody",
  },
  {
    id: "telemetry-charts",
    targetTestId: "vn-telemetry-charts",
    titleKey: "navigation.tour.chartsTitle",
    bodyKey: "navigation.tour.chartsBody",
  },
  {
    id: "pre-arm",
    targetTestId: "vn-pre-arm",
    titleKey: "navigation.tour.preArmTitle",
    bodyKey: "navigation.tour.preArmBody",
  },
];

/**
 * localStorage key the orchestrator uses to record that the tour has
 * been completed at least once. A single key per browser; the plugin
 * context carries no per-drone identifier, so the tour persists across
 * drones.
 */
export const TOUR_PERSIST_KEY = "vision-nav-tour-seen";
