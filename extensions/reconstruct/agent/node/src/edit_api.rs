//! The editor's read-only owner routes over a product version, answered from
//! the full-resolution cloud:
//!
//! - `POST /api/compute/edit/estimate` `{job, source, format, decimate?}` →
//!   `{points, bytes}`: the predicted size of an export or decimation.
//! - `POST /api/compute/edit/volume` `{job, source, polygon, base_z, cell_m?}`
//!   → `{volume_m3, cells, cell_m}`: the volume above a base plane inside a
//!   ground polygon.

use std::path::{Path, PathBuf};

use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;

use ados_mapping_protocol::edit::{valid_polygon, valid_source, DecimateTarget, ExportFormat};

use crate::products::cloud::read_laz;
use crate::recon::{edit_export, valid_job_id, StageError};

/// Grid cell of a volume measurement when the request names none.
pub const DEFAULT_CELL_M: f64 = 0.25;

#[derive(Debug, Deserialize)]
pub struct EstimateRequest {
    pub job: String,
    pub source: String,
    pub format: ExportFormat,
    #[serde(default)]
    pub decimate: Option<DecimateTarget>,
}

#[derive(Debug, Deserialize)]
pub struct VolumeRequest {
    pub job: String,
    pub source: String,
    pub polygon: Vec<[f64; 2]>,
    pub base_z: f64,
    #[serde(default)]
    pub cell_m: Option<f64>,
}

fn refuse(status: StatusCode, code: &str, detail: impl Into<String>) -> Response {
    (
        status,
        Json(serde_json::json!({ "error": code, "detail": detail.into() })),
    )
        .into_response()
}

/// The product job directory and the source path in it, or the refusal.
fn product(work_root: &Path, job: &str, source: &str) -> Result<PathBuf, Box<Response>> {
    if !valid_job_id(job) {
        return Err(Box::new(refuse(
            StatusCode::NOT_FOUND,
            "E_NOT_FOUND",
            "unknown job",
        )));
    }
    valid_source(source)
        .map_err(|e| Box::new(refuse(StatusCode::BAD_REQUEST, "E_INPUT", e.to_string())))?;
    let dir = work_root.join("jobs").join(job);
    if !dir.join(source).is_file() {
        return Err(Box::new(refuse(
            StatusCode::NOT_FOUND,
            "E_SOURCE_MISSING",
            "The source artifact is missing on this node.",
        )));
    }
    Ok(dir)
}

fn stage_refusal(e: StageError) -> Response {
    let status = if e.code == "E_INPUT" {
        StatusCode::BAD_REQUEST
    } else {
        StatusCode::INTERNAL_SERVER_ERROR
    };
    refuse(status, &e.code, e.message)
}

pub async fn estimate(work_root: &Path, req: EstimateRequest) -> Response {
    let dir = match product(work_root, &req.job, &req.source) {
        Ok(d) => d,
        Err(r) => return *r,
    };
    if let Some(Err(e)) = req.decimate.as_ref().map(DecimateTarget::validate) {
        return refuse(StatusCode::BAD_REQUEST, "E_INPUT", e.to_string());
    }
    let done = tokio::task::spawn_blocking(move || {
        edit_export::estimate_size(&dir, &req.source, req.format, req.decimate.as_ref())
    })
    .await;
    match done {
        Ok(Ok((points, bytes))) => {
            Json(serde_json::json!({ "points": points, "bytes": bytes })).into_response()
        }
        Ok(Err(e)) => stage_refusal(e),
        Err(_) => refuse(
            StatusCode::INTERNAL_SERVER_ERROR,
            "E_INTERNAL",
            "the estimate did not finish",
        ),
    }
}

pub async fn volume(work_root: &Path, req: VolumeRequest) -> Response {
    let dir = match product(work_root, &req.job, &req.source) {
        Ok(d) => d,
        Err(r) => return *r,
    };
    if let Err(e) = valid_polygon(&req.polygon) {
        return refuse(StatusCode::BAD_REQUEST, "E_INPUT", e.to_string());
    }
    let cell = req.cell_m.unwrap_or(DEFAULT_CELL_M);
    if !(cell.is_finite() && cell > 0.0) || !req.base_z.is_finite() {
        return refuse(
            StatusCode::BAD_REQUEST,
            "E_INPUT",
            "The cell size and base height are finite, and the cell above zero.",
        );
    }
    let done = tokio::task::spawn_blocking(move || {
        read_laz(&dir.join(&req.source))
            .map(|c| edit_export::volume_above(&c, &req.polygon, req.base_z, cell))
    })
    .await;
    match done {
        Ok(Ok((v, cells))) => {
            Json(serde_json::json!({ "volume_m3": v, "cells": cells, "cell_m": cell }))
                .into_response()
        }
        Ok(Err(e)) => refuse(StatusCode::INTERNAL_SERVER_ERROR, "E_IO", e.to_string()),
        Err(_) => refuse(
            StatusCode::INTERNAL_SERVER_ERROR,
            "E_INTERNAL",
            "the measurement did not finish",
        ),
    }
}
