//! Rig rules the generated structural types cannot express.

use crate::{Error, Result, Rig, RigSensorKind};

impl Rig {
    /// Check the rules the schema states conditionally: every `lidar` sensor
    /// carries its `lidar` scan model.
    pub fn check(&self) -> Result<()> {
        for s in &self.sensors {
            if s.kind == RigSensorKind::Lidar && s.lidar.is_none() {
                return Err(Error::Invalid(format!(
                    "rig sensor {}: a lidar sensor needs a \"lidar\" model block",
                    s.id
                )));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::Rig;

    fn rig(lidar: Option<serde_json::Value>) -> Rig {
        let mut sensor = serde_json::json!({
            "id": "lidar0", "kind": "lidar", "role": "up", "time_offset_ns": 0,
            "T_body_sensor": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
        });
        if let Some(l) = lidar {
            sensor["lidar"] = l;
        }
        serde_json::from_value(serde_json::json!({
            "contract": "ados.capture/1", "rig_id": "r", "calib_version": "1",
            "sensors": [sensor]
        }))
        .unwrap()
    }

    #[test]
    fn lidar_sensor_without_model_is_rejected() {
        let err = rig(None).check().unwrap_err().to_string();
        assert!(
            err.contains("rig sensor lidar0: a lidar sensor needs a \"lidar\" model block"),
            "{err}"
        );
    }

    #[test]
    fn lidar_sensor_with_model_passes() {
        let model = serde_json::json!({
            "model": "rplidar-c1", "pattern": "planar", "h_fov_deg": 360.0,
            "v_min_deg": 0.0, "v_max_deg": 0.0, "rate_hz": 10.0, "points_per_s": 5000.0,
            "min_range_m": 0.05, "max_range_m": 12.0, "range_sigma_m": 0.03,
            "blocked_azimuth_deg": [[130.0, 230.0]]
        });
        rig(Some(model)).check().unwrap();
    }
}
