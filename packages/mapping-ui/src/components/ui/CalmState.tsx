/**
 * @module CalmState
 * @description A centred icon and one line explaining why a view has nothing
 * to show (node not paired, unreachable, no peers). Never an error surface.
 */

import type { LucideIcon } from "lucide-react";

export function CalmState({ icon: Icon, message }: { icon: LucideIcon; message: string }) {
  return (
    <div className="map:flex map:h-full map:min-h-[280px] map:items-center map:justify-center map:p-6">
      <div className="map:text-center">
        <Icon className="map:mx-auto map:mb-2 map:h-5 map:w-5 map:text-text-tertiary" />
        <p className="map:max-w-sm map:text-[11px] map:text-text-tertiary">{message}</p>
      </div>
    </div>
  );
}
