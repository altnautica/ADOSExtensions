//! Frame conversions for the map's inputs. The map frame is `map_ned`
//! (north, east, down); poses carry forward-right-down body or sensor axes.
//! LiDAR drivers and the rig mount are forward-left-up (`F = diag(1, -1, -1)`
//! turns one into the other).

use ados_mapping_pose::se3;
use ados_voxmap::Pose;

/// A forward-left-up vector as forward-right-down.
#[inline]
pub fn flu_to_frd(p: [f32; 3]) -> [f32; 3] {
    [p[0], -p[1], -p[2]]
}

/// Whether sensor point `p` (forward-left-up) lies in a blocked sector.
#[inline]
pub fn blocked(p: &[f32; 3], sectors: &[[f64; 2]]) -> bool {
    !sectors.is_empty()
        && ados_scan2d::scan::is_blocked(
            f64::from(p[1]).atan2(f64::from(p[0])).to_degrees(),
            sectors,
        )
}

/// A pose from a row-major rotation (forward-right-down axes into the map)
/// and a translation.
pub fn pose_of(r: &[f64; 9], t: [f64; 3]) -> Pose {
    Pose {
        t: t.map(|v| v as f32),
        q: se3::quat_from_matrix(r).map(|v| v as f32),
    }
}

/// The rig mount (body-from-sensor, forward-left-up 4x4) as a pose with
/// forward-right-down axes: the sensor's place and attitude on the body.
pub fn mount_pose(mount: &[f64; 16]) -> Pose {
    let (r, t) = se3::from_mat4(mount);
    pose_of(
        &crate::nav::frames::flu_to_frd_rot(&r),
        crate::nav::frames::flu_to_frd_vec(t),
    )
}

/// The body pose from the sensor's pose in the map and its `mount` on the
/// body ([`mount_pose`]).
pub fn body_from_sensor(sensor: &Pose, mount: &Pose) -> Pose {
    let [aw, ax, ay, az] = sensor.q;
    // The mount's inverse rotation.
    let [bw, bx, by, bz] = [mount.q[0], -mount.q[1], -mount.q[2], -mount.q[3]];
    let q = [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ];
    let off = Pose { t: [0.0; 3], q }.rotate(mount.t);
    Pose {
        t: [0, 1, 2].map(|a| sensor.t[a] - off[a]),
        q,
    }
}
