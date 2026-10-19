//! The job store: a single-writer SQLite database holding datasets, jobs, and
//! outputs. Mirrors the durable single-writer pattern the logging daemon uses.
//! Open `:memory:` for tests and a file path on a node.

use rusqlite::{params, Connection, Row};
use serde::{Deserialize, Serialize};
use world_engine_protocol::compute::{Dataset, JobRecord, Output};

use crate::{ComputeError, ComputeJobState};

fn enum_to_db<T: Serialize>(v: &T) -> Result<String, ComputeError> {
    match serde_json::to_value(v)? {
        serde_json::Value::String(s) => Ok(s),
        other => Err(ComputeError::Backend {
            backend: "store".into(),
            message: format!("expected a string-serialized enum, got {other}"),
        }),
    }
}

fn enum_from_db<T: for<'de> Deserialize<'de>>(s: &str) -> Result<T, ComputeError> {
    Ok(serde_json::from_value(serde_json::Value::String(
        s.to_string(),
    ))?)
}

/// Map an INSERT error: a UNIQUE-constraint violation (a duplicate id) becomes a
/// `Conflict` so a retrying client gets a 409, not an opaque store fault.
fn store_or_conflict(e: rusqlite::Error, ctx: String) -> ComputeError {
    if let rusqlite::Error::SqliteFailure(f, _) = &e {
        if f.code == rusqlite::ErrorCode::ConstraintViolation {
            return ComputeError::Conflict(ctx);
        }
    }
    ComputeError::Store(e)
}

/// The job store over one SQLite connection.
pub struct JobStore {
    pub(crate) conn: Connection,
}

impl JobStore {
    /// Open (and migrate) a store at `path`. Pass `:memory:` for an ephemeral
    /// in-memory store.
    pub fn open(path: &str) -> Result<Self, ComputeError> {
        let conn = Connection::open(path)?;
        Self::from_conn(conn)
    }

    /// Open an in-memory store (tests).
    pub fn open_in_memory() -> Result<Self, ComputeError> {
        Self::from_conn(Connection::open_in_memory()?)
    }

    fn from_conn(conn: Connection) -> Result<Self, ComputeError> {
        conn.execute_batch(
            "PRAGMA journal_mode=WAL;
             CREATE TABLE IF NOT EXISTS datasets (
                 id TEXT PRIMARY KEY,
                 kind TEXT NOT NULL,
                 created_ms INTEGER NOT NULL,
                 meta TEXT NOT NULL
             );
             CREATE TABLE IF NOT EXISTS jobs (
                 id TEXT PRIMARY KEY,
                 kind TEXT NOT NULL,
                 dataset_id TEXT,
                 state TEXT NOT NULL,
                 progress REAL NOT NULL,
                 params TEXT NOT NULL,
                 result_ref TEXT,
                 error TEXT,
                 created_ms INTEGER NOT NULL,
                 updated_ms INTEGER NOT NULL
             );
             CREATE INDEX IF NOT EXISTS jobs_state_created ON jobs(state, created_ms);
             CREATE TABLE IF NOT EXISTS outputs (
                 id TEXT PRIMARY KEY,
                 job_id TEXT NOT NULL,
                 kind TEXT NOT NULL,
                 uri TEXT NOT NULL,
                 meta TEXT NOT NULL DEFAULT 'null',
                 created_ms INTEGER NOT NULL
             );
             CREATE INDEX IF NOT EXISTS outputs_job ON outputs(job_id);
             CREATE TABLE IF NOT EXISTS job_stages (
                 job_id TEXT NOT NULL,
                 stage TEXT NOT NULL,
                 seq INTEGER NOT NULL,
                 state TEXT NOT NULL,
                 progress REAL NOT NULL DEFAULT 0,
                 started_ns INTEGER,
                 ended_ns INTEGER,
                 detail TEXT NOT NULL DEFAULT 'null',
                 PRIMARY KEY (job_id, stage)
             );
             CREATE TABLE IF NOT EXISTS job_errors (
                 job_id TEXT PRIMARY KEY,
                 error TEXT NOT NULL
             );
             CREATE TABLE IF NOT EXISTS job_tools (
                 job_id TEXT NOT NULL,
                 tool TEXT NOT NULL,
                 version TEXT NOT NULL,
                 PRIMARY KEY (job_id, tool)
             );",
        )?;
        Ok(Self { conn })
    }

    // --- datasets ---------------------------------------------------------

    pub fn insert_dataset(&self, d: &Dataset) -> Result<(), ComputeError> {
        self.conn
            .execute(
                "INSERT INTO datasets (id, kind, created_ms, meta) VALUES (?1, ?2, ?3, ?4)",
                params![d.id, d.kind, d.created_ms, serde_json::to_string(&d.meta)?],
            )
            .map_err(|e| store_or_conflict(e, format!("dataset {} already exists", d.id)))?;
        Ok(())
    }

    pub fn get_dataset(&self, id: &str) -> Result<Option<Dataset>, ComputeError> {
        let mut stmt = self
            .conn
            .prepare("SELECT id, kind, created_ms, meta FROM datasets WHERE id = ?1")?;
        let mut rows = stmt.query(params![id])?;
        match rows.next()? {
            Some(row) => Ok(Some(Dataset {
                id: row.get(0)?,
                kind: row.get(1)?,
                created_ms: row.get(2)?,
                meta: serde_json::from_str(&row.get::<_, String>(3)?)?,
            })),
            None => Ok(None),
        }
    }

    // --- jobs -------------------------------------------------------------

    /// Insert a new job (typically in [`ComputeJobState::Queued`]).
    pub fn submit_job(&self, j: &JobRecord) -> Result<(), ComputeError> {
        self.conn.execute(
            "INSERT INTO jobs
                (id, kind, dataset_id, state, progress, params, result_ref, error, created_ms, updated_ms)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10)",
            params![
                j.id,
                enum_to_db(&j.kind)?,
                j.dataset_id,
                enum_to_db(&j.state)?,
                j.progress,
                serde_json::to_string(&j.params)?,
                j.result_ref,
                j.error,
                j.created_ms,
                j.updated_ms,
            ],
        )
        .map_err(|e| store_or_conflict(e, format!("job {} already exists", j.id)))?;
        Ok(())
    }

    /// Count jobs in a given state. Cheap and indexed (`jobs_state_created`), so
    /// the heartbeat can poll queue depth without loading the whole table.
    pub fn count_in_state(&self, state: ComputeJobState) -> Result<u32, ComputeError> {
        let n: i64 = self.conn.query_row(
            "SELECT COUNT(*) FROM jobs WHERE state = ?1",
            params![enum_to_db(&state)?],
            |row| row.get(0),
        )?;
        Ok(n as u32)
    }

    fn row_to_job(row: &Row) -> Result<JobRecord, ComputeError> {
        Ok(JobRecord {
            id: row.get(0)?,
            kind: enum_from_db(&row.get::<_, String>(1)?)?,
            dataset_id: row.get(2)?,
            state: enum_from_db(&row.get::<_, String>(3)?)?,
            progress: row.get(4)?,
            params: serde_json::from_str(&row.get::<_, String>(5)?)?,
            result_ref: row.get(6)?,
            error: row.get(7)?,
            created_ms: row.get(8)?,
            updated_ms: row.get(9)?,
        })
    }

    const JOB_COLS: &'static str =
        "id, kind, dataset_id, state, progress, params, result_ref, error, created_ms, updated_ms";

    /// Atomically claim the oldest queued job: mark it `Running` and return the
    /// updated row. A single UPDATE...RETURNING statement, so the claim and the
    /// state flip cannot race; the returned job already has `state = Running`.
    /// Returns `None` when the queue is empty. This is the worker's claim step,
    /// run under the engine lock; the backend then runs WITHOUT the lock.
    pub fn claim_next_queued(&self, now_ms: i64) -> Result<Option<JobRecord>, ComputeError> {
        let sql = format!(
            "UPDATE jobs SET state = 'running', updated_ms = ?1
             WHERE id = (SELECT id FROM jobs WHERE state = 'queued' ORDER BY created_ms ASC LIMIT 1)
             RETURNING {}",
            Self::JOB_COLS
        );
        let mut stmt = self.conn.prepare(&sql)?;
        let mut rows = stmt.query(params![now_ms])?;
        match rows.next()? {
            Some(row) => Ok(Some(Self::row_to_job(row)?)),
            None => Ok(None),
        }
    }

    /// The oldest queued job, or `None` if the queue is empty. This is the
    /// scheduler's pick; a real multi-worker node would claim it atomically.
    pub fn next_queued_job(&self) -> Result<Option<JobRecord>, ComputeError> {
        let sql = format!(
            "SELECT {} FROM jobs WHERE state = 'queued' ORDER BY created_ms ASC LIMIT 1",
            Self::JOB_COLS
        );
        let mut stmt = self.conn.prepare(&sql)?;
        let mut rows = stmt.query([])?;
        match rows.next()? {
            Some(row) => Ok(Some(Self::row_to_job(row)?)),
            None => Ok(None),
        }
    }

    pub fn get_job(&self, id: &str) -> Result<Option<JobRecord>, ComputeError> {
        let sql = format!("SELECT {} FROM jobs WHERE id = ?1", Self::JOB_COLS);
        let mut stmt = self.conn.prepare(&sql)?;
        let mut rows = stmt.query(params![id])?;
        match rows.next()? {
            Some(row) => Ok(Some(Self::row_to_job(row)?)),
            None => Ok(None),
        }
    }

    pub fn list_jobs(&self) -> Result<Vec<JobRecord>, ComputeError> {
        let sql = format!(
            "SELECT {} FROM jobs ORDER BY created_ms ASC",
            Self::JOB_COLS
        );
        let mut stmt = self.conn.prepare(&sql)?;
        let rows = stmt.query_map([], |row| {
            Ok(Self::row_to_job(row).map_err(|e| {
                rusqlite::Error::FromSqlConversionFailure(0, rusqlite::types::Type::Text, e.into())
            }))
        })?;
        let mut out = Vec::new();
        for r in rows {
            out.push(r??);
        }
        Ok(out)
    }

    /// Update a job's running state and result. Returns `NotFound` if no row
    /// matched.
    pub fn set_job_state(
        &self,
        id: &str,
        state: ComputeJobState,
        progress: f32,
        result_ref: Option<&str>,
        error: Option<&str>,
        updated_ms: i64,
    ) -> Result<(), ComputeError> {
        let n = self.conn.execute(
            "UPDATE jobs SET state = ?2, progress = ?3, result_ref = ?4, error = ?5, updated_ms = ?6
             WHERE id = ?1",
            params![id, enum_to_db(&state)?, progress, result_ref, error, updated_ms],
        )?;
        if n == 0 {
            return Err(ComputeError::NotFound(format!("job {id}")));
        }
        Ok(())
    }

    /// Cancel a job if it has not reached a terminal state. Returns whether a
    /// row was cancelled.
    pub fn cancel_job(&self, id: &str, updated_ms: i64) -> Result<bool, ComputeError> {
        let n = self.conn.execute(
            "UPDATE jobs SET state = 'cancelled', updated_ms = ?2
             WHERE id = ?1 AND state IN ('queued', 'running')",
            params![id, updated_ms],
        )?;
        Ok(n > 0)
    }

    /// Requeue any job left in `Running` (e.g. the daemon crashed mid-backend so
    /// the job never reached a terminal state). Called once at startup so a
    /// crash does not strand a job forever: a `Running` job is neither claimable
    /// (claim picks only `queued`) nor purgeable (retention drops only terminal
    /// states). A staged job's finished stages are kept, so it resumes at the
    /// stage that was interrupted. Returns the number requeued.
    pub fn requeue_stale_running(&self, updated_ms: i64) -> Result<usize, ComputeError> {
        let n = self.conn.execute(
            "UPDATE jobs SET state = 'queued', updated_ms = ?1
             WHERE state = 'running'",
            params![updated_ms],
        )?;
        self.conn.execute(
            "UPDATE job_stages SET state = 'pending', progress = 0, started_ns = NULL
             WHERE state = 'running'",
            [],
        )?;
        Ok(n)
    }

    // --- outputs ----------------------------------------------------------

    pub fn insert_output(&self, o: &Output) -> Result<(), ComputeError> {
        self.conn.execute(
            "INSERT INTO outputs (id, job_id, kind, uri, meta, created_ms)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
            params![
                o.id,
                o.job_id,
                o.kind,
                o.uri,
                serde_json::to_string(&o.meta)?,
                o.created_ms
            ],
        )?;
        Ok(())
    }

    pub fn outputs_for_job(&self, job_id: &str) -> Result<Vec<Output>, ComputeError> {
        let mut stmt = self.conn.prepare(
            "SELECT id, job_id, kind, uri, meta, created_ms FROM outputs WHERE job_id = ?1
             ORDER BY created_ms ASC",
        )?;
        let rows = stmt.query_map(params![job_id], |row| {
            let meta_s: String = row.get(4)?;
            Ok(Output {
                id: row.get(0)?,
                job_id: row.get(1)?,
                kind: row.get(2)?,
                uri: row.get(3)?,
                meta: serde_json::from_str(&meta_s).unwrap_or(serde_json::Value::Null),
                created_ms: row.get(5)?,
            })
        })?;
        let mut out = Vec::new();
        for r in rows {
            out.push(r?);
        }
        Ok(out)
    }

    /// Retention: drop terminal jobs (and their outputs) updated before
    /// `cutoff_ms`. Returns the number of jobs removed.
    pub fn purge_terminal_before(&self, cutoff_ms: i64) -> Result<usize, ComputeError> {
        self.conn.execute(
            "DELETE FROM outputs WHERE job_id IN
                (SELECT id FROM jobs
                 WHERE state IN ('completed','failed','cancelled') AND updated_ms < ?1)",
            params![cutoff_ms],
        )?;
        let n = self.conn.execute(
            "DELETE FROM jobs
             WHERE state IN ('completed','failed','cancelled') AND updated_ms < ?1",
            params![cutoff_ms],
        )?;
        self.conn.execute_batch(
            "DELETE FROM job_stages WHERE job_id NOT IN (SELECT id FROM jobs);
             DELETE FROM job_errors WHERE job_id NOT IN (SELECT id FROM jobs);
             DELETE FROM job_tools WHERE job_id NOT IN (SELECT id FROM jobs);",
        )?;
        Ok(n)
    }
}

#[cfg(test)]
#[path = "store_tests.rs"]
mod tests;
