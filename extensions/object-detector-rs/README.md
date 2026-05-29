# ADOS Object Detector (Rust)

A trivial Rust object detector that proves the shared vision frame bus end to
end. It is the first Rust vision extension and exists to validate the frame and
detection path on any board, with no machine-learning model and no native
dependency.

## What it does

The plugin subscribes to the vision engine's normalized frames over
`ctx.vision.subscribe_frames`. For each frame it runs a cheap brightness
heuristic and publishes one bounding box on the detection topic via
`ctx.vision.publish_one`:

1. Read the frame as a luma image.
2. Scan it in 64x64 cells and pick the cell with the highest mean luma.
3. Emit a 64x64 box at that cell, labelled `bright-region`, with confidence set
   to the cell's mean luma scaled to `0.0..=1.0`.

A frame smaller than one cell yields no detection.

## Luma per format

- `nv12` and `yuv420p` are planar; the luma plane is the first `width * height`
  bytes, read directly.
- `rgb24` is packed three bytes per pixel; luma is approximated per pixel from
  the R, G, B bytes with the Rec. 601 weights.

## Permissions

- `vision.frame.read` to subscribe to engine frames.
- `vision.detection.publish` to publish the boxes it derives.

It does not register a model (`vision.model.register` is not requested): the
heuristic runs in the plugin, so there is no engine-run model to load.

## Threading

`subscribe_frames` invokes the callback on the IPC reader task, which must not
block. The callback computes the detection synchronously (cheap) and offloads
the async publish to a spawned task, holding a cheap clone of the vision client.

## Build and test

The detector is a Cargo workspace member. The heuristic is a pure function, so
a host build and the unit tests run anywhere:

```bash
# From the repository root.
cargo test -p object-detector-rs        # runs the frame-bus + heuristic tests
cargo build -p object-detector-rs       # host check that it compiles + links
```

The unit tests drive the real frame read path through
`ados_sdk::testing::FakeVisionEngine`: a synthetic frame with a known bright
patch is written into a real ring and resolved through the seqlock exactly as
production does, then `detect` asserts the box lands on the patch.

For a board build, cross-compile to the static aarch64 musl target and pack the
archive:

```bash
scripts/build-rust.sh object-detector-rs
scripts/pack-rust.sh object-detector-rs
```

See `docs/rust-plugins.md` for the toolchain and packing details.
