//! A deterministic synthetic capture session: a textured room rendered by CPU
//! ray casting from a pinhole camera on a smooth walk, with the ground-truth
//! body poses recorded on `/pose/vio`.
//!
//! The room is 8 x 6 x 3 m (ENU, Z up) with two columns and three boxes near
//! the walls. Every face carries its own value-noise texture with coloured
//! discs and squares on top, so SIFT finds many distinct features. The body
//! (FLU) walks an elliptical loop at 1.5 m height, yawing to look outward and
//! nodding a few degrees in pitch and roll so the intrinsics stay observable;
//! the camera looks along body +x (OpenCV axes through the rig's
//! `T_body_sensor`).

use std::path::{Path, PathBuf};

use ados_capture_contract::foxglove::{Pose, PoseInFrame, Quaternion, RawImage, Vector3};
use ados_capture_contract::*;

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
/// 90 degree horizontal field of view.
pub const FX: f64 = 320.0;
pub const FY: f64 = 320.0;
const CX: f64 = 320.0;
const CY: f64 = 240.0;
/// Camera frames over one loop (about 0.2 m apart).
pub const FRAMES: u64 = 56;
/// Frame period (5 Hz); poses are recorded at four times that rate.
const FRAME_NS: u64 = 200_000_000;
const POSES_PER_FRAME: u64 = 4;
const T0: u64 = 1_000_000_000;
/// Supersampling per pixel axis.
const SS: usize = 2;

const ROOM: Aabb = Aabb {
    min: [0.0, 0.0, 0.0],
    max: [8.0, 6.0, 3.0],
};
const OBJECTS: [Aabb; 5] = [
    // Two floor-to-ceiling columns.
    Aabb {
        min: [1.0, 1.0, 0.0],
        max: [1.4, 1.4, 3.0],
    },
    Aabb {
        min: [6.6, 4.6, 0.0],
        max: [7.0, 5.0, 3.0],
    },
    // Three boxes.
    Aabb {
        min: [6.2, 0.6, 0.0],
        max: [7.2, 1.5, 0.9],
    },
    Aabb {
        min: [0.7, 4.4, 0.0],
        max: [1.7, 5.4, 1.2],
    },
    Aabb {
        min: [3.5, 5.2, 0.0],
        max: [4.5, 5.7, 0.6],
    },
];

/// Camera looking along body +x (OpenCV axes: x right = -y body, y down = -z
/// body), row-major.
const T_BODY_CAM: [f64; 16] = [
    0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

struct Aabb {
    min: [f64; 3],
    max: [f64; 3],
}

/// Body position and orientation (unit quaternion w, x, y, z) at loop
/// parameter `s` in [0, 1].
fn body_at(s: f64) -> ([f64; 3], [f64; 4]) {
    let th = std::f64::consts::TAU * s;
    let pos = [
        4.0 + 2.2 * th.cos(),
        3.0 + 1.4 * th.sin(),
        1.5 + 0.05 * (3.0 * th).sin(),
    ];
    // Look outward, swaying around the radial direction.
    let yaw = th + 0.25 * (2.0 * th).sin();
    let pitch = 0.1 * (5.0 * th).sin();
    let roll = 0.05 * (7.0 * th + 1.0).sin();
    // q = q_z(yaw) * q_y(pitch) * q_x(roll).
    let (sy, cy) = (yaw / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sr, cr) = (roll / 2.0).sin_cos();
    let q = [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ];
    (pos, q)
}

/// The rotation matrix (row-major rows) of a unit quaternion (w, x, y, z).
fn rotation(q: [f64; 4]) -> [[f64; 3]; 3] {
    let [w, x, y, z] = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn loop_param(t_ns: u64) -> f64 {
    (t_ns - T0) as f64 / (FRAMES * FRAME_NS) as f64
}

fn mix(mut h: u64) -> u64 {
    h ^= h >> 33;
    h = h.wrapping_mul(0xff51_afd7_ed55_8ccd);
    h ^= h >> 33;
    h = h.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
    h ^ (h >> 33)
}

fn hash(a: i64, b: i64, salt: u64) -> u64 {
    mix((a as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)
        ^ (b as u64).wrapping_mul(0xc2b2_ae3d_27d4_eb4f)
        ^ salt.wrapping_mul(0x1656_67b1_9e37_79f9))
}

/// Uniform in [0, 1) from a hash.
fn unit(h: u64) -> f64 {
    (h >> 11) as f64 / (1u64 << 53) as f64
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn value_noise(u: f64, v: f64, salt: u64) -> f64 {
    let (iu, iv) = (u.floor(), v.floor());
    let fade = |x: f64| x * x * (3.0 - 2.0 * x);
    let (su, sv) = (fade(u - iu), fade(v - iv));
    let (iu, iv) = (iu as i64, iv as i64);
    let at = |du: i64, dv: i64| unit(hash(iu + du, iv + dv, salt));
    lerp(
        lerp(at(0, 0), at(1, 0), su),
        lerp(at(0, 1), at(1, 1), su),
        sv,
    )
}

/// Blob layers: grid cell, share of cells with a blob, radius range (m).
const LAYERS: [(f64, f64, f64, f64); 3] = [
    (0.35, 0.6, 0.04, 0.12),
    (0.14, 0.55, 0.015, 0.05),
    (0.06, 0.45, 0.007, 0.02),
];

/// The colour of surface `surface` at its in-plane coordinates (u, v), metres.
fn albedo(u: f64, v: f64, surface: u64) -> [f64; 3] {
    let sh = mix(surface.wrapping_add(0x5eed));
    let (u, v) = (u + 97.0 * unit(sh), v + 89.0 * unit(mix(sh)));
    let base = [
        0.4 + 0.6 * unit(mix(sh ^ 1)),
        0.4 + 0.6 * unit(mix(sh ^ 2)),
        0.4 + 0.6 * unit(mix(sh ^ 3)),
    ];
    // Five octaves of value noise, 0.4 m down to about 2 cm, contrast
    // stretched around mid grey.
    let (mut n, mut amp, mut sum, mut freq) = (0.0, 1.0, 0.0, 2.5);
    for octave in 0..5 {
        n += amp * value_noise(u * freq, v * freq, sh ^ (octave + 10));
        sum += amp;
        amp *= 0.6;
        freq *= 2.1;
    }
    let lum = 0.55 + 1.8 * (n / sum - 0.5);
    let mut c = base.map(|b| b * lum);
    // Discs and squares at three scales, each inside its own grid cell, dark
    // or bright so they stand out in grey.
    for (layer, &(cell, share, r_min, r_max)) in LAYERS.iter().enumerate() {
        let (cu, cv) = ((u / cell).floor(), (v / cell).floor());
        let h = hash(cu as i64, cv as i64, sh ^ (0xb10b + layer as u64));
        if unit(h) >= share {
            continue;
        }
        let r = r_min + (r_max - r_min) * unit(mix(h ^ 3));
        let bu = (cu * cell) + r + (cell - 2.0 * r) * unit(mix(h ^ 1));
        let bv = (cv * cell) + r + (cell - 2.0 * r) * unit(mix(h ^ 2));
        let (du, dv) = ((u - bu).abs(), (v - bv).abs());
        let d = if h & 1 == 0 {
            (du * du + dv * dv).sqrt()
        } else {
            du.max(dv)
        };
        let a = ((r - d) / (0.25 * r).min(0.01)).clamp(0.0, 1.0);
        if a > 0.0 {
            let level = if h & 2 == 0 {
                0.05 + 0.25 * unit(mix(h ^ 7))
            } else {
                0.7 + 0.3 * unit(mix(h ^ 7))
            };
            let hue = [unit(mix(h ^ 4)), unit(mix(h ^ 5)), unit(mix(h ^ 6))];
            let mean = (hue[0] + hue[1] + hue[2]) / 3.0 + 1e-3;
            for k in 0..3 {
                c[k] = lerp(c[k], (hue[k] / mean * level).min(1.0), a);
            }
        }
    }
    c
}

/// A ray hit: distance, the face's normal axis and sign, the surface id.
struct Hit {
    t: f64,
    axis: usize,
    positive: bool,
    surface: u64,
}

fn cast(o: [f64; 3], d: [f64; 3]) -> Hit {
    // The room seen from inside: the nearest wall in front.
    let mut best = Hit {
        t: f64::INFINITY,
        axis: 0,
        positive: true,
        surface: 0,
    };
    for k in 0..3 {
        if d[k].abs() < 1e-12 {
            continue;
        }
        let (plane, positive) = if d[k] > 0.0 {
            (ROOM.max[k], false)
        } else {
            (ROOM.min[k], true)
        };
        let t = (plane - o[k]) / d[k];
        if t > 0.0 && t < best.t {
            best = Hit {
                t,
                axis: k,
                positive,
                surface: (k * 2 + positive as usize) as u64,
            };
        }
    }
    // The objects seen from outside (slab test).
    for (i, b) in OBJECTS.iter().enumerate() {
        let (mut t_in, mut t_out, mut axis) = (f64::NEG_INFINITY, f64::INFINITY, 0);
        for k in 0..3 {
            if d[k].abs() < 1e-12 {
                if o[k] < b.min[k] || o[k] > b.max[k] {
                    t_out = f64::NEG_INFINITY;
                }
                continue;
            }
            let (a, c) = ((b.min[k] - o[k]) / d[k], (b.max[k] - o[k]) / d[k]);
            let (near, far) = if a < c { (a, c) } else { (c, a) };
            if near > t_in {
                t_in = near;
                axis = k;
            }
            t_out = t_out.min(far);
        }
        if t_in > 0.0 && t_in < t_out && t_in < best.t {
            let positive = d[axis] < 0.0;
            best = Hit {
                t: t_in,
                axis,
                positive,
                surface: (6 + i * 6 + axis * 2 + positive as usize) as u64,
            };
        }
    }
    best
}

/// Fixed, view-independent shading per face orientation.
fn shade(axis: usize, positive: bool) -> f64 {
    match (axis, positive) {
        (2, true) => 0.95,
        (2, false) => 0.7,
        (0, true) => 0.85,
        (0, false) => 0.78,
        (1, true) => 0.9,
        _ => 0.72,
    }
}

/// Render the RGB frame seen from body position `pos` with orientation `q`.
fn render(pos: [f64; 3], q: [f64; 4]) -> Vec<u8> {
    let r = rotation(q);
    let (w, h) = (WIDTH as usize, HEIGHT as usize);
    let mut out = vec![0u8; w * h * 3];
    for j in 0..h {
        for i in 0..w {
            let mut acc = [0.0; 3];
            for sj in 0..SS {
                for si in 0..SS {
                    let x = (i as f64 + (si as f64 + 0.5) / SS as f64 - CX) / FX;
                    let y = (j as f64 + (sj as f64 + 0.5) / SS as f64 - CY) / FY;
                    // OpenCV camera -> FLU body -> world.
                    let b = [1.0, -x, -y];
                    let d = r.map(|row| row[0] * b[0] + row[1] * b[1] + row[2] * b[2]);
                    let hit = cast(pos, d);
                    let p = [
                        pos[0] + hit.t * d[0],
                        pos[1] + hit.t * d[1],
                        pos[2] + hit.t * d[2],
                    ];
                    let (u, v) = (p[(hit.axis + 1) % 3], p[(hit.axis + 2) % 3]);
                    let c = albedo(u, v, hit.surface);
                    let s = shade(hit.axis, hit.positive);
                    for k in 0..3 {
                        acc[k] += c[k] * s;
                    }
                }
            }
            let px = &mut out[(j * w + i) * 3..][..3];
            for k in 0..3 {
                let v = acc[k] / (SS * SS) as f64;
                px[k] = (v.clamp(0.0, 1.0) * 255.0).round() as u8;
            }
        }
    }
    out
}

fn rig() -> Rig {
    Rig {
        contract: ContractId::AdosCapture1,
        rig_id: "synth_rig".into(),
        calib_version: "1".into(),
        sensors: vec![RigSensor {
            id: "cam0".into(),
            kind: RigSensorKind::Camera,
            role: RigSensorRole::FrontLeft,
            t_body_sensor: T_BODY_CAM.into(),
            camera: Some(CameraModel {
                model: CameraModelModel::Pinhole,
                width: WIDTH,
                height: HEIGHT,
                fx: FX,
                fy: FY,
                cx: CX,
                cy: CY,
                dist: vec![],
                calibrated: true,
                readout_ns: 0,
            }),
            lidar: None,
            imu_noise: None,
            time_offset_ns: 0,
            lever_arm_m: None,
        }],
    }
}

fn pose_msg(pos: [f64; 3], q: [f64; 4]) -> PoseInFrame {
    PoseInFrame {
        timestamp: None,
        frame_id: "world".into(),
        pose: Some(Pose {
            position: Some(Vector3 {
                x: pos[0],
                y: pos[1],
                z: pos[2],
            }),
            orientation: Some(Quaternion {
                w: q[0],
                x: q[1],
                y: q[2],
                z: q[3],
            }),
        }),
    }
}

/// Write the session under `root`; returns the session directory.
pub fn build(root: &Path) -> PathBuf {
    let end_ns = T0 + FRAMES * FRAME_NS;
    let manifest = Manifest {
        contract: ContractId::AdosCapture1,
        session_id: session_id("sim-1", 1_792_368_000, 0x5e),
        device_id: "sim-1".into(),
        profile: ManifestProfile::Sim,
        timebase: Timebase::Sim,
        t0_ns: T0,
        utc_at_t0_ns: None,
        sigma_ns: -1,
        anchor: None,
        rig: RigRef {
            rig_id: "synth_rig".into(),
            calib_version: "1".into(),
        },
        operator: Operator {
            site: Some("Synthetic room".into()),
            floor: Some(0),
            rooms: vec![Room {
                name: "Room".into(),
                floor: 0,
                t_start_ns: T0,
                t_end_ns: end_ns,
            }],
        },
        segments: vec![],
        health: SessionHealth {
            blur: false,
            texture: false,
            flicker: false,
            corridor: false,
            frames_dropped_pct: 0.0,
        },
    };
    // Render every frame in parallel, then record in time order.
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get());
    let mut frames: Vec<Vec<u8>> = vec![Vec::new(); FRAMES as usize];
    std::thread::scope(|s| {
        let chunk = (FRAMES as usize).div_ceil(threads);
        for (c, slots) in frames.chunks_mut(chunk).enumerate() {
            s.spawn(move || {
                for (k, slot) in slots.iter_mut().enumerate() {
                    let i = (c * chunk + k) as u64;
                    let (pos, q) = body_at(loop_param(T0 + i * FRAME_NS));
                    *slot = render(pos, q);
                }
            });
        }
    });
    let mut w = SessionWriter::create(root, manifest, &rig()).unwrap();
    let pose_ns = FRAME_NS / POSES_PER_FRAME;
    for (i, data) in frames.into_iter().enumerate() {
        let t = T0 + i as u64 * FRAME_NS;
        let (pos, q) = body_at(loop_param(t));
        w.write_proto(&topics::pose("vio"), t, &pose_msg(pos, q))
            .unwrap();
        let img = RawImage {
            timestamp: None,
            frame_id: "cam0".into(),
            width: WIDTH,
            height: HEIGHT,
            encoding: "rgb8".into(),
            step: WIDTH * 3,
            data,
        };
        w.write_proto(&topics::cam_image("cam0"), t, &img).unwrap();
        for k in 1..POSES_PER_FRAME {
            let tp = t + k * pose_ns;
            let (pos, q) = body_at(loop_param(tp));
            w.write_proto(&topics::pose("vio"), tp, &pose_msg(pos, q))
                .unwrap();
        }
    }
    let dir = w.dir().to_path_buf();
    w.finish().unwrap();
    dir
}
