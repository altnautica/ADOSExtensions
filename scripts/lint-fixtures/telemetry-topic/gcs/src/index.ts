export function start(ctx: any) {
  return ctx.telemetry.subscribe("gimbal", () => {});
}
