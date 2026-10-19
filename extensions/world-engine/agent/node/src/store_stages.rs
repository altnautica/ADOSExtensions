//! Persistence for staged jobs: one row per (job, stage) so a job interrupted
//! by a node restart resumes at the stage that was running, plus the
//! structured error a failed staged job carries.

use rusqlite::{params, OptionalExtension};
use serde::{Deserialize, Serialize};

use crate::recon::StageError;
use crate::{ComputeError, JobStore, Output};

/// A stage's lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum StageState {
    Pending,
    Running,
    Done,
    Failed,
    Skipped,
}

impl StageState {
    fn as_str(self) -> &'static str {
        match self {
            Self::Pending => "pending",
            Self::Running => "running",
            Self::Done => "done",
            Self::Failed => "failed",
            Self::Skipped => "skipped",
        }
    }

    fn parse(s: &str) -> Self {
        match s {
            "running" => Self::Running,
            "done" => Self::Done,
            "failed" => Self::Failed,
            "skipped" => Self::Skipped,
            _ => Self::Pending,
        }
    }

    /// Whether a resumed job may skip this stage.
    pub fn is_finished(self) -> bool {
        matches!(self, Self::Done | Self::Skipped)
    }
}

/// One persisted stage of a job, as served on the job status.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StageRow {
    pub id: String,
    pub state: StageState,
    pub progress: f32,
    pub started_ns: Option<i64>,
    pub ended_ns: Option<i64>,
    /// Stage facts (tool versions used, counts, which trainer ran).
    #[serde(default)]
    pub detail: serde_json::Value,
}

impl JobStore {
    /// Create the stage rows for a job once, in order. Existing rows (a
    /// resumed job) are kept as they are.
    pub fn init_stages(&self, job_id: &str, stages: &[&str]) -> Result<(), ComputeError> {
        for (seq, stage) in stages.iter().enumerate() {
            self.conn.execute(
                "INSERT OR IGNORE INTO job_stages (job_id, stage, seq, state) VALUES (?1, ?2, ?3, 'pending')",
                params![job_id, stage, seq as i64],
            )?;
        }
        Ok(())
    }

    /// The stage rows of a job in pipeline order (empty for a one-shot job).
    pub fn stages(&self, job_id: &str) -> Result<Vec<StageRow>, ComputeError> {
        let mut stmt = self.conn.prepare(
            "SELECT stage, state, progress, started_ns, ended_ns, detail FROM job_stages
             WHERE job_id = ?1 ORDER BY seq ASC",
        )?;
        let rows = stmt.query_map(params![job_id], |row| {
            let detail: String = row.get(5)?;
            Ok(StageRow {
                id: row.get(0)?,
                state: StageState::parse(&row.get::<_, String>(1)?),
                progress: row.get::<_, f64>(2)? as f32,
                started_ns: row.get(3)?,
                ended_ns: row.get(4)?,
                detail: serde_json::from_str(&detail).unwrap_or(serde_json::Value::Null),
            })
        })?;
        rows.collect::<Result<Vec<_>, _>>().map_err(Into::into)
    }

    /// Move a stage to `state`. Entering `Running` stamps `started_ns`; any
    /// finished state stamps `ended_ns`. `detail`, when given, replaces it.
    pub fn set_stage(
        &self,
        job_id: &str,
        stage: &str,
        state: StageState,
        now_ns: i64,
        detail: Option<&serde_json::Value>,
    ) -> Result<(), ComputeError> {
        let progress = if state.is_finished() { 1.0 } else { 0.0 };
        let (started, ended) = match state {
            StageState::Running => (Some(now_ns), None),
            StageState::Pending => (None, None),
            _ => (None, Some(now_ns)),
        };
        self.conn.execute(
            "UPDATE job_stages SET state = ?3,
                 progress = CASE WHEN ?3 = 'failed' THEN progress ELSE ?4 END,
                 started_ns = COALESCE(?5, CASE WHEN ?3 = 'pending' THEN NULL ELSE started_ns END),
                 ended_ns = ?6
             WHERE job_id = ?1 AND stage = ?2",
            params![job_id, stage, state.as_str(), progress, started, ended],
        )?;
        if let Some(d) = detail {
            self.conn.execute(
                "UPDATE job_stages SET detail = ?3 WHERE job_id = ?1 AND stage = ?2",
                params![job_id, stage, serde_json::to_string(d)?],
            )?;
        }
        Ok(())
    }

    /// Record a running stage's progress and the job's overall progress.
    pub fn set_stage_progress(
        &self,
        job_id: &str,
        stage: &str,
        stage_progress: f32,
        job_progress: f32,
        now_ms: i64,
    ) -> Result<(), ComputeError> {
        self.conn.execute(
            "UPDATE job_stages SET progress = ?3 WHERE job_id = ?1 AND stage = ?2",
            params![job_id, stage, stage_progress.clamp(0.0, 1.0) as f64],
        )?;
        self.conn.execute(
            "UPDATE jobs SET progress = ?2, updated_ms = ?3 WHERE id = ?1 AND state = 'running'",
            params![job_id, job_progress.clamp(0.0, 1.0) as f64, now_ms],
        )?;
        Ok(())
    }

    /// Attach a structured error to a job (shown as `error` on its status).
    pub fn set_job_error(&self, job_id: &str, e: &StageError) -> Result<(), ComputeError> {
        self.conn.execute(
            "INSERT INTO job_errors (job_id, error) VALUES (?1, ?2)
             ON CONFLICT(job_id) DO UPDATE SET error = excluded.error",
            params![job_id, serde_json::to_string(e)?],
        )?;
        Ok(())
    }

    /// The structured error of a job, if one was recorded.
    pub fn job_error(&self, job_id: &str) -> Result<Option<StageError>, ComputeError> {
        let raw: Option<String> = self
            .conn
            .query_row(
                "SELECT error FROM job_errors WHERE job_id = ?1",
                params![job_id],
                |r| r.get(0),
            )
            .optional()?;
        Ok(raw.and_then(|s| serde_json::from_str(&s).ok()))
    }

    /// Drop a job's outputs (a resumed staged job lists its deliverables anew).
    pub fn delete_outputs(&self, job_id: &str) -> Result<(), ComputeError> {
        self.conn
            .execute("DELETE FROM outputs WHERE job_id = ?1", params![job_id])?;
        Ok(())
    }

    /// Drop a job's structured error (a retried job).
    pub fn clear_job_error(&self, job_id: &str) -> Result<(), ComputeError> {
        self.conn
            .execute("DELETE FROM job_errors WHERE job_id = ?1", params![job_id])?;
        Ok(())
    }

    /// The outputs of the newest completed session reconstruction of
    /// `session_id` (empty when it was never reconstructed).
    pub fn latest_session_outputs(&self, session_id: &str) -> Result<Vec<Output>, ComputeError> {
        let job_id: Option<String> = self
            .conn
            .query_row(
                "SELECT id FROM jobs
                 WHERE kind = 'reconstruct' AND state = 'completed'
                   AND json_extract(params, '$.session_id') = ?1
                   AND json_extract(params, '$.preset') IS NOT NULL
                 ORDER BY updated_ms DESC LIMIT 1",
                params![session_id],
                |r| r.get(0),
            )
            .optional()?;
        match job_id {
            Some(id) => self.outputs_for_job(&id),
            None => Ok(Vec::new()),
        }
    }

    /// How many completed session reconstructions of `session_id` exist: the
    /// next one's placement `generation`.
    pub fn session_generation(&self, session_id: &str) -> Result<u32, ComputeError> {
        let n: i64 = self.conn.query_row(
            "SELECT COUNT(*) FROM jobs
             WHERE kind = 'reconstruct' AND state = 'completed'
               AND json_extract(params, '$.session_id') = ?1",
            params![session_id],
            |r| r.get(0),
        )?;
        Ok(n as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ComputeJobKind, ComputeJobState, JobRecord};

    fn job(id: &str) -> JobRecord {
        JobRecord {
            id: id.into(),
            kind: ComputeJobKind::Reconstruct,
            dataset_id: None,
            state: ComputeJobState::Queued,
            progress: 0.0,
            params: serde_json::json!({"session_id": "s1", "preset": "hq"}),
            result_ref: None,
            error: None,
            created_ms: 1,
            updated_ms: 1,
        }
    }

    #[test]
    fn stages_persist_in_order_and_survive_a_requeue() {
        let s = JobStore::open_in_memory().unwrap();
        s.submit_job(&job("j")).unwrap();
        s.init_stages("j", &["prepare", "sfm", "train"]).unwrap();
        s.claim_next_queued(2).unwrap();
        s.set_stage("j", "prepare", StageState::Running, 10, None)
            .unwrap();
        s.set_stage(
            "j",
            "prepare",
            StageState::Done,
            20,
            Some(&serde_json::json!({"frames": 2})),
        )
        .unwrap();
        s.set_stage("j", "sfm", StageState::Running, 30, None)
            .unwrap();
        s.set_stage_progress("j", "sfm", 0.5, 0.3, 3).unwrap();
        // Init again (a resumed job) keeps the existing rows.
        s.init_stages("j", &["prepare", "sfm", "train"]).unwrap();
        // A crash: the running stage goes back to pending, the done one stays.
        s.requeue_stale_running(40).unwrap();
        let rows = s.stages("j").unwrap();
        assert_eq!(
            rows.iter().map(|r| r.id.as_str()).collect::<Vec<_>>(),
            ["prepare", "sfm", "train"]
        );
        assert_eq!(rows[0].state, StageState::Done);
        assert_eq!(rows[0].started_ns, Some(10));
        assert_eq!(rows[0].ended_ns, Some(20));
        assert_eq!(rows[0].detail["frames"], 2);
        assert_eq!(rows[1].state, StageState::Pending);
        assert_eq!(rows[1].started_ns, None);
        assert_eq!(rows[2].state, StageState::Pending);
    }

    #[test]
    fn a_job_error_round_trips_and_clears() {
        let s = JobStore::open_in_memory().unwrap();
        let e = StageError::tool_missing("colmap", "not installed");
        s.set_job_error("j", &e).unwrap();
        assert_eq!(s.job_error("j").unwrap(), Some(e));
        s.clear_job_error("j").unwrap();
        assert_eq!(s.job_error("j").unwrap(), None);
    }

    #[test]
    fn latest_session_outputs_pick_the_newest_completed_reconstruction() {
        let s = JobStore::open_in_memory().unwrap();
        for (id, t) in [("old", 5), ("new", 9)] {
            let mut j = job(id);
            j.updated_ms = t;
            s.submit_job(&j).unwrap();
            s.set_job_state(id, ComputeJobState::Completed, 1.0, None, None, t)
                .unwrap();
            s.insert_output(&Output::new(
                format!("{id}-o"),
                id.into(),
                "pointcloud".into(),
                format!("file:///{id}.laz"),
                t,
            ))
            .unwrap();
        }
        let outs = s.latest_session_outputs("s1").unwrap();
        assert_eq!(outs[0].job_id, "new");
        assert_eq!(s.session_generation("s1").unwrap(), 2);
        assert!(s.latest_session_outputs("other").unwrap().is_empty());
    }
}
