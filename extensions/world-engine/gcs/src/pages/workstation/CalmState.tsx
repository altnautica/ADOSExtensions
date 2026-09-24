/**
 * @module CalmState
 * @description A centred icon and one line explaining why a workstation view
 * has nothing to show (node not paired, unreachable, no peers). Never an
 * error surface.
 */

import type { LucideIcon } from "lucide-react";

export function CalmState({ icon: Icon, message }: { icon: LucideIcon; message: string }) {
  return (
    <div className="we:flex we:h-full we:min-h-[280px] we:items-center we:justify-center we:p-6">
      <div className="we:text-center">
        <Icon className="we:mx-auto we:mb-2 we:h-5 we:w-5 we:text-text-tertiary" />
        <p className="we:max-w-sm we:text-[11px] we:text-text-tertiary">{message}</p>
      </div>
    </div>
  );
}
