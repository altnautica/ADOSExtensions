import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { TourApp } from "../tour/TourApp";

const root = document.getElementById("root");
if (root) {
  createRoot(root).render(
    <StrictMode>
      <TourApp />
    </StrictMode>,
  );
}
