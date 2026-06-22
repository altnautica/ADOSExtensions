/**
 * Follow-state subscription hook.
 *
 * The agent half publishes its follow read-back on FOLLOW_STATE_TOPIC; the
 * host forwards it to the iframe as a one-way event. Each iframe instance
 * owns its own subscription lifecycle (there is no shared store in the
 * sandbox). The raw payload is snake_case off the wire and normalized to
 * the camelCase FollowState here.
 *
 * @license GPL-3.0-or-later
 */

import { useEffect, useState } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import {
  EMPTY_FOLLOW_STATE,
  FOLLOW_STATE_TOPIC,
  normalizeFollowState,
  type FollowState,
  type RawFollowState,
} from "./types";

/** Subscribe to the agent follow read-back and surface the latest state. */
export function useFollowState(ctx: PluginContext): FollowState {
  const [state, setState] = useState<FollowState>(EMPTY_FOLLOW_STATE);

  useEffect(() => {
    const off = ctx.events.subscribe<RawFollowState>(
      FOLLOW_STATE_TOPIC,
      (raw) => {
        setState(normalizeFollowState(raw));
      },
    );
    return off;
  }, [ctx]);

  return state;
}
