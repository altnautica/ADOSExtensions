//! Small helpers on the generated geometry types.

use crate::schema::{
    ArtifactDescriptor, ArtifactDescriptorFormat, ArtifactDescriptorKind, CameraModel,
    CameraModelModel, ContractId, Placement,
};

/// Row-major 3x3 identity.
pub const IDENTITY_3X3: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

impl Placement {
    /// An artifact already expressed in the ENU frame at `anchor`, unit scale.
    pub fn identity(anchor: Option<crate::GeoAnchor>) -> Self {
        Self {
            anchor,
            r_enu_artifact: IDENTITY_3X3,
            t_enu_artifact: [0.0; 3],
            scale: 1.0,
        }
    }

    /// Map a point from artifact coordinates into ENU metres.
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let r = &self.r_enu_artifact;
        let t = &self.t_enu_artifact;
        let s = self.scale;
        [
            s * (r[0] * p[0] + r[1] * p[1] + r[2] * p[2]) + t[0],
            s * (r[3] * p[0] + r[4] * p[1] + r[5] * p[2]) + t[1],
            s * (r[6] * p[0] + r[7] * p[1] + r[8] * p[2]) + t[2],
        ]
    }
}

impl ArtifactDescriptor {
    /// The descriptor for `file` placed by `placement`.
    pub fn new(
        kind: ArtifactDescriptorKind,
        file: impl Into<String>,
        format: ArtifactDescriptorFormat,
        sha256: impl Into<String>,
        placement: Placement,
        generation: u32,
        source_session: impl Into<String>,
    ) -> Self {
        Self {
            contract: ContractId::AdosCapture1,
            kind,
            file: file.into(),
            format,
            sha256: sha256.into(),
            anchor: placement.anchor,
            r_enu_artifact: placement.r_enu_artifact,
            t_enu_artifact: placement.t_enu_artifact,
            scale: placement.scale,
            generation,
            source_session: source_session.into(),
        }
    }

    /// The placement fields as a [`Placement`].
    pub fn placement(&self) -> Placement {
        Placement {
            anchor: self.anchor.clone(),
            r_enu_artifact: self.r_enu_artifact,
            t_enu_artifact: self.t_enu_artifact,
            scale: self.scale,
        }
    }
}

impl CameraModel {
    /// Number of `dist` coefficients the model defines.
    pub fn expected_dist_len(model: CameraModelModel) -> usize {
        match model {
            CameraModelModel::Pinhole => 0,
            CameraModelModel::Opencv => 5,
            CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => 4,
        }
    }

    /// Whether `dist` has the length `model` requires.
    pub fn dist_len_ok(&self) -> bool {
        self.dist.len() == Self::expected_dist_len(self.model)
    }

    /// Row-major 3x3 intrinsic matrix K.
    pub fn k_matrix(&self) -> [f64; 9] {
        [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn placement_applies_rotation_scale_translation() {
        // Y-up artifact into Z-up ENU: x -> x, y -> z, z -> -y.
        let p = Placement {
            anchor: None,
            r_enu_artifact: [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
            t_enu_artifact: [10.0, 0.0, 0.0],
            scale: 2.0,
        };
        assert_eq!(p.apply([1.0, 2.0, 3.0]), [12.0, -6.0, 4.0]);
        assert_eq!(
            Placement::identity(None).apply([1.0, 2.0, 3.0]),
            [1.0, 2.0, 3.0]
        );
    }

    #[test]
    fn descriptor_carries_its_placement() {
        let p = Placement::identity(None);
        let d = ArtifactDescriptor::new(
            ArtifactDescriptorKind::Pointcloud,
            "cloud.laz",
            ArtifactDescriptorFormat::Laz,
            "00".repeat(32),
            p.clone(),
            1,
            "s",
        );
        assert_eq!(d.placement(), p);
    }
}
