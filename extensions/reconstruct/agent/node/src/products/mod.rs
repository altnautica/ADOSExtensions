//! Indoor products built from a finished reconstruction, all in the local ENU
//! metre frame (Z up): point-cloud I/O, floor plans, rack aisles, cloud change
//! detection, mesh measurement and the walkthrough tour bundle.

use std::path::PathBuf;

pub mod aisles;
pub mod align;
pub mod change;
pub mod cloud;
pub mod floorplan;
pub mod geo;
pub mod measure;
pub mod tour;

mod auto_rooms;
mod carve;
mod floorplan_svg;
mod geom;
mod grid2d;
mod ply;
mod rooms;
mod skeleton;
mod slice;
mod walls;

#[cfg(test)]
mod tests;

pub use cloud::{PointCloud, TriMesh};

/// Failure of a product reader, writer or bundler.
#[derive(Debug, thiserror::Error)]
pub enum ProductError {
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    #[error("format error: {0}")]
    Format(String),
    #[error("viewer bundle missing: {} has no index.html", .0.display())]
    ViewerMissing(PathBuf),
    /// Two clouds could not be brought into one frame.
    #[error("frames unrelated: {0}")]
    FramesUnrelated(String),
    /// Two clouds align equally well at two headings about 180° apart.
    #[error("alignment ambiguous: {0}")]
    AlignAmbiguous(String),
}
