//! Reconstruction presets: what each one runs and which pinned tools it needs.

use serde::{Deserialize, Serialize};

/// A session reconstruction preset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Preset {
    /// Field preview: COLMAP poses and a short Brush splat.
    Preview,
    /// High fidelity: COLMAP + gsplat (MCMC, 3DGUT when needed, bilateral
    /// grid) + mesh + point cloud.
    Hq,
    /// `hq` plus the floorplan, rooms, hero stills and the static tour bundle.
    Tour,
    /// `hq` over large sites (hierarchical mapping) plus floorplan and aisles.
    Warehouse,
}

/// What a preset runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PresetSpec {
    /// Keyframe selection: minimum baseline between selected frames of one
    /// camera, metres.
    pub min_baseline_m: f64,
    /// Keyframe selection: minimum rotation between selected frames, degrees.
    pub min_rotation_deg: f64,
    /// Upper bound on exported images over every camera.
    pub max_images: usize,
    /// Training iterations.
    pub train_steps: u64,
    /// Splat cap for an 8 GB GPU (scaled by VRAM by the train stage).
    pub max_splats_8gb: u64,
    /// The mesh stage runs.
    pub mesh: bool,
    /// Masks (dynamic objects, mirrors/glass) are computed.
    pub masks: bool,
    /// Monocular depth priors are computed for camera-only sessions.
    pub depth_priors: bool,
    /// Floorplan + rooms.
    pub floorplan: bool,
    /// Rack/aisle detection.
    pub aisles: bool,
    /// Hero stills + the tour bundle.
    pub tour: bool,
    /// Spark LoD (`.rad`) output.
    pub lod: bool,
    /// COLMAP hierarchical mapping for unposed sessions.
    pub hierarchical: bool,
}

impl Preset {
    pub fn parse(s: &str) -> Option<Self> {
        serde_json::from_value(serde_json::Value::String(s.to_string())).ok()
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Preview => "preview",
            Self::Hq => "hq",
            Self::Tour => "tour",
            Self::Warehouse => "warehouse",
        }
    }

    pub fn spec(self) -> PresetSpec {
        let hq = PresetSpec {
            min_baseline_m: 0.10,
            min_rotation_deg: 6.0,
            max_images: 1500,
            train_steps: 30_000,
            max_splats_8gb: 1_000_000,
            mesh: true,
            masks: true,
            depth_priors: true,
            floorplan: false,
            aisles: false,
            tour: false,
            lod: false,
            hierarchical: false,
        };
        match self {
            Self::Preview => PresetSpec {
                min_baseline_m: 0.25,
                min_rotation_deg: 12.0,
                max_images: 300,
                train_steps: 7_000,
                max_splats_8gb: 500_000,
                mesh: false,
                masks: false,
                depth_priors: false,
                ..hq
            },
            Self::Hq => hq,
            Self::Tour => PresetSpec {
                floorplan: true,
                tour: true,
                lod: true,
                ..hq
            },
            Self::Warehouse => PresetSpec {
                max_images: 6000,
                floorplan: true,
                aisles: true,
                lod: true,
                hierarchical: true,
                ..hq
            },
        }
    }

    /// The pinned tools the preset cannot run without. Checked when the job
    /// is submitted and again when it starts (`E_TOOL_MISSING`). Brush is
    /// required by every preset: it is the preview trainer and the stand-in
    /// when the gsplat kernels do not run. A tour also needs the built tour
    /// viewer it bundles.
    pub fn required_tools(self) -> Vec<&'static str> {
        let spec = self.spec();
        let mut tools = vec!["colmap", "splat-transform", "brush"];
        if self != Self::Preview {
            tools.push("gsplat_train");
        }
        if spec.lod {
            tools.push("spark-build-lod");
        }
        if spec.tour {
            tools.push(TOUR_VIEWER);
        }
        tools
    }
}

/// The pseudo-tool name of the built static tour viewer.
pub const TOUR_VIEWER: &str = "tour-viewer";

/// The stages of a session reconstruction, in order.
pub const RECONSTRUCT_STAGES: [&str; 5] = ["prepare", "sfm", "train", "mesh", "products"];
/// The stages of a change job.
pub const CHANGE_STAGES: [&str; 2] = ["prepare", "products"];
/// The stages of a calibration job.
pub const CALIBRATE_STAGES: [&str; 2] = ["prepare", "calibrate"];

/// A stage's share of a session reconstruction's overall progress.
pub fn stage_weight(stage: &str) -> f32 {
    match stage {
        "prepare" => 0.05,
        "sfm" => 0.25,
        "train" => 0.45,
        "mesh" => 0.15,
        "products" => 0.10,
        _ => 0.5,
    }
}

/// Per-job switches that can only turn a preset's optional products off (job
/// param `products`, e.g. `{"stills": false, "lod": false}`); an absent key
/// keeps what the preset makes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProductSwitches {
    /// Hero stills and the static tour bundle built from them.
    pub stills: bool,
    /// The Spark LoD (`.rad`) splat.
    pub lod: bool,
}

impl Default for ProductSwitches {
    fn default() -> Self {
        Self {
            stills: true,
            lod: true,
        }
    }
}

impl ProductSwitches {
    /// Read the `products` param (`None` = the preset's products).
    pub fn parse(v: Option<&serde_json::Value>) -> Result<Self, String> {
        let mut s = Self::default();
        let Some(v) = v else { return Ok(s) };
        let obj = v
            .as_object()
            .ok_or("`products` must be an object like {\"stills\": false, \"lod\": false}")?;
        for (k, val) in obj {
            let on = val
                .as_bool()
                .ok_or_else(|| format!("`products.{k}` must be true or false"))?;
            match k.as_str() {
                "stills" => s.stills = on,
                "lod" => s.lod = on,
                _ => return Err(format!("unknown product switch `{k}` (stills, lod)")),
            }
        }
        Ok(s)
    }

    /// `spec` with the switched-off products removed.
    pub fn apply(self, spec: PresetSpec) -> PresetSpec {
        PresetSpec {
            tour: spec.tour && self.stills,
            lod: spec.lod && self.lod,
            ..spec
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_parse_and_name_their_tools() {
        assert_eq!(Preset::parse("tour"), Some(Preset::Tour));
        assert_eq!(Preset::parse("bogus"), None);
        assert!(Preset::Preview.required_tools().contains(&"brush"));
        assert!(!Preset::Preview.spec().mesh);
        let wh = Preset::Warehouse.required_tools();
        assert!(wh.contains(&"gsplat_train") && wh.contains(&"spark-build-lod"));
        assert!(Preset::Warehouse.spec().aisles && Preset::Tour.spec().tour);
        let total: f32 = RECONSTRUCT_STAGES.iter().map(|s| stage_weight(s)).sum();
        assert!((total - 1.0).abs() < 1e-6);
    }

    #[test]
    fn product_switches_only_turn_preset_products_off() {
        use serde_json::json;
        let tour = Preset::Tour.spec();
        let off = ProductSwitches::parse(Some(&json!({"stills": false, "lod": false}))).unwrap();
        let s = off.apply(tour);
        assert!(!s.tour && !s.lod && s.floorplan && s.mesh && s.train_steps == tour.train_steps);
        assert_eq!(ProductSwitches::parse(None).unwrap().apply(tour), tour);
        let on = ProductSwitches::parse(Some(&json!({"lod": true}))).unwrap();
        assert!(
            !on.apply(Preset::Hq.spec()).lod,
            "a switch cannot add a product"
        );
        assert!(ProductSwitches::parse(Some(&json!({"mesh": false}))).is_err());
        assert!(ProductSwitches::parse(Some(&json!({"lod": "no"}))).is_err());
        assert!(ProductSwitches::parse(Some(&json!(false))).is_err());
    }
}
