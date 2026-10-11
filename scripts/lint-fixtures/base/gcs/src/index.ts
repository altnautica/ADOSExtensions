const STATE_CHANNEL = "status";

export function start(ctx: any) {
  return ctx.telemetry
    .subscribe<unknown>(STATE_CHANNEL, () => {});
}
