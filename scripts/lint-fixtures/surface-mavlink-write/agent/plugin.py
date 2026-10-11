import serial


async def run(ctx):
    port = serial.Serial("/dev/ttyS0")
    async for frame in ctx.vision.subscribe_frames():
        speed = await ctx.config_kv.get("speed", 1.0)
        boost = await ctx.config_kv.get("boost", False)
        await ctx.config_kv.set("last_frame", frame.ts)
        seen = await ctx.config_kv.get("last_frame", None)
        del speed, boost, seen
