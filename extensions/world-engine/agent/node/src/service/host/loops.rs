//! The host-service background loops: retention, compute status, the cloud
//! job mirror and the Atlas event receiver.

use super::*;
/// Periodically reclaim terminal jobs older than the retention window.
pub(super) async fn retention_loop(state: Arc<Mutex<Engine>>, retention_ms: i64) {
    loop {
        tokio::time::sleep(Duration::from_secs(60)).await;
        let engine = state.lock().await;
        match engine
            .scheduler()
            .store()
            .purge_terminal_before(now_ms() - retention_ms)
        {
            Ok(n) if n > 0 => tracing::info!(removed = n, "retention purge"),
            Ok(_) => {}
            Err(e) => tracing::warn!(error = %e, "retention purge failed"),
        }
    }
}

/// Every 5 s, snapshot the engine heartbeat + the host-GPU block, keep it as
/// the latest compute status (`GET /compute/status`), and publish it on the
/// `status` telemetry channel when a host is connected. The GPU read shells out
/// to `system_profiler`/`powermetrics`, so it runs on a blocking thread to keep
/// the async runtime responsive; a join failure degrades to an all-null GPU
/// block. Best-effort: a store or publish error is logged, never fatal.
pub(super) async fn heartbeat_loop(
    state: Arc<Mutex<Engine>>,
    latest: Arc<LatestComputeStatus>,
    ctx: Option<PluginContext>,
) {
    loop {
        tokio::time::sleep(HEARTBEAT_INTERVAL).await;
        let now = now_ms();
        let hb = state.lock().await.heartbeat();
        let hb = match hb {
            Ok(hb) => hb,
            Err(e) => {
                tracing::warn!(error = %e, "compute heartbeat snapshot failed");
                continue;
            }
        };
        let gpu = tokio::task::spawn_blocking(world_engine_node::gpu::sample)
            .await
            .unwrap_or_default();
        let status = ComputeHeartbeatSidecar::from_heartbeat(&hb, gpu, now);
        latest.set(status.clone());
        let Some(ctx) = &ctx else { continue };
        match rmpv::ext::to_value(status_payload(&status)) {
            Ok(payload) => {
                if let Err(e) = ctx.telemetry.extend(STATUS_CHANNEL, payload).await {
                    tracing::warn!(error = %e, "status telemetry publish failed");
                }
            }
            Err(e) => tracing::warn!(error = %e, "status telemetry encode failed"),
        }
    }
}

/// Every [`JOBS_SYNC_INTERVAL`], put each reconstruct-job record that is new or
/// changed since the cloud last accepted it (collection `jobs`, key = job id,
/// subject = the capturing drone). A put the cloud did not accept (the relay is
/// down on an unpaired or offline node) is logged at debug and retried on the
/// next tick.
pub(super) async fn jobs_sync_loop(
    state: Arc<Mutex<Engine>>,
    ctx: PluginContext,
    compute_node_id: String,
) {
    let mut sync = JobsCloudSync::new();
    loop {
        tokio::time::sleep(JOBS_SYNC_INTERVAL).await;
        let jobs = {
            let engine = state.lock().await;
            build_atlas_jobs(engine.scheduler().store())
        };
        let jobs = match jobs {
            Ok(jobs) => jobs,
            Err(e) => {
                tracing::warn!(error = %e, "compute jobs snapshot failed");
                continue;
            }
        };
        for record in sync.pending(&jobs, &compute_node_id) {
            let data = match rmpv::ext::to_value(&record.data) {
                Ok(v) => v,
                Err(e) => {
                    tracing::warn!(job = %record.key, error = %e, "job record encode failed");
                    continue;
                }
            };
            match ctx
                .cloud
                .put_record(JOBS_COLLECTION, &record.key, data, Some(&record.device_id))
                .await
            {
                Ok(reply) if put_accepted(&reply) => {
                    tracing::debug!(job = %record.key, "job record synced to cloud");
                    sync.mark_synced(record);
                }
                Ok(reply) => {
                    tracing::debug!(job = %record.key, reply = %reply, "job record not accepted; retrying next tick")
                }
                Err(e) => {
                    tracing::debug!(job = %record.key, error = %e, "job record put failed; retrying next tick")
                }
            }
        }
    }
}

/// Drain the Atlas event receiver into the job queue. One [`AtlasIngest`] lives
/// for the task: it persists each keyframe's image to the work-root dataset (no
/// store, no lock) and, on the terminal `Bagged` state, finalizes the dataset
/// (writes `transforms.json`) and yields the reconstruct job. The disk writes run
/// lock-free; the engine lock is held only briefly for the store submit (the same
/// lock-briefly discipline the worker uses). A malformed frame is swallowed; a
/// real filesystem or store fault is logged and the loop continues. The loop ends
/// when the event channel closes (the receiver router dropped its senders), which
/// only happens on shutdown.
///
/// When live reconstruction is enabled, the loop ALSO runs the per-session cadence
/// ([`run_live_cycles`]) on a tick and after each keyframe: it periodically
/// snapshots the growing capture and enqueues a real reconstruct so the world
/// model updates during the flight, not only at the end. In both modes a slow
/// sweep finalizes any session that went silent without a `Bagged` frame (see
/// [`AtlasIngest::idle_bags`]), so a capture whose bag was lost is still
/// reconstructed and its state released.
pub(super) async fn atlas_receiver_loop(
    mut rx: mpsc::Receiver<AtlasEvent>,
    state: Arc<Mutex<Engine>>,
    work_root: PathBuf,
    live_config: LiveReconstructConfig,
) {
    let mut ingest = AtlasIngest::with_live_config(work_root, live_config);

    let mut tick = tokio::time::interval(Duration::from_secs(LIVE_CADENCE_TICK_SECS));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut idle_sweep = tokio::time::interval(IDLE_SWEEP);
    idle_sweep.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            maybe_event = rx.recv() => {
                let Some(event) = maybe_event else { break };
                drain_event(&mut ingest, &state, &event).await;
                // A keyframe may have hit the count trigger; check promptly.
                if live_config.enabled {
                    run_live_cycles(&mut ingest, &state).await;
                }
            }
            _ = tick.tick(), if live_config.enabled => {
                // The interval trigger + the skip-while-running reconcile.
                run_live_cycles(&mut ingest, &state).await;
            }
            _ = idle_sweep.tick() => match ingest.idle_bags(now_ms()) {
                Ok(bags) => {
                    for (dataset, job) in bags {
                        let engine = state.lock().await;
                        match submit_reconstruct_job(engine.scheduler().store(), &dataset, &job) {
                            Ok(job_id) => tracing::info!(job = %job_id, "atlas idle session finalized: reconstruct job enqueued"),
                            Err(e) => tracing::error!(error = %e, "atlas reconstruct submit failed"),
                        }
                    }
                }
                Err(e) => tracing::error!(error = %e, "atlas idle session finalize failed"),
            },
        }
    }
    tracing::info!("atlas receiver loop ended (event channel closed)");
}

/// Handle one received Atlas event: persist a keyframe or, on the terminal bag,
/// enqueue the final reconstruct. A malformed frame is swallowed; a real disk or
/// store fault is logged and the caller continues.
pub(super) async fn drain_event(
    ingest: &mut AtlasIngest,
    state: &Arc<Mutex<Engine>>,
    event: &AtlasEvent,
) {
    match ingest.step(event, now_ms()) {
        Ok(Some((dataset, job))) => {
            let keyframes_received = ingest.keyframes_seen();
            let submitted = {
                let engine = state.lock().await;
                submit_reconstruct_job(engine.scheduler().store(), &dataset, &job)
            };
            match submitted {
                Ok(job_id) => tracing::info!(
                    job = %job_id,
                    keyframes_received,
                    "atlas capture bagged: reconstruct job enqueued"
                ),
                Err(e) => tracing::error!(error = %e, "atlas reconstruct submit failed"),
            }
        }
        Ok(None) => {}
        Err(e) => tracing::error!(error = %e, "atlas ingest disk write failed"),
    }
}

/// Drive the live-reconstruction cadence: reconcile the skip-while-running guard
/// against the store (a cycle whose job reached a terminal state, or vanished,
/// releases the guard), then enqueue a fresh periodic reconstruct for every
/// session now due. The store is the source of truth for "is the cycle done", so
/// cycles coalesce instead of piling up. A snapshot or submit fault is logged; the
/// cadence keeps running.
pub(super) async fn run_live_cycles(ingest: &mut AtlasIngest, state: &Arc<Mutex<Engine>>) {
    // 1. Release the guard for any session whose in-flight cycle finished.
    let in_flight = ingest.in_flight_cycles();
    if !in_flight.is_empty() {
        let finished: Vec<String> = {
            let engine = state.lock().await;
            let store = engine.scheduler().store();
            in_flight
                .into_iter()
                .filter_map(|(session, job_id)| match store.get_job(&job_id) {
                    Ok(Some(job)) if is_terminal(job.state) => Some(session),
                    // The job was purged (retention) — treat as finished so the
                    // session is never stuck with a guard that never releases.
                    Ok(None) => Some(session),
                    Ok(Some(_)) => None, // still queued / running: keep the guard
                    Err(e) => {
                        tracing::warn!(error = %e, "live cycle reconcile read failed");
                        None
                    }
                })
                .collect()
        };
        for session in finished {
            ingest.note_cycle_finished(&session);
        }
    }

    // 2. Enqueue a fresh periodic reconstruct for every session now due.
    match ingest.due_reconstructs(now_ms()) {
        Ok(jobs) => {
            for (dataset, job) in jobs {
                let submitted = {
                    let engine = state.lock().await;
                    submit_reconstruct_job(engine.scheduler().store(), &dataset, &job)
                };
                let keyframes = dataset
                    .meta
                    .get("keyframes")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0);
                match submitted {
                    Ok(id) => tracing::info!(
                        job = %id,
                        keyframes,
                        "atlas live reconstruct cycle enqueued"
                    ),
                    Err(e) => tracing::error!(error = %e, "atlas live reconstruct submit failed"),
                }
            }
        }
        Err(e) => tracing::error!(error = %e, "atlas live snapshot write failed"),
    }
}

/// Whether a job has reached a terminal state (the live cadence's
/// skip-while-running guard releases on a terminal in-flight cycle).
pub(super) fn is_terminal(state: ComputeJobState) -> bool {
    matches!(
        state,
        ComputeJobState::Completed | ComputeJobState::Failed | ComputeJobState::Cancelled
    )
}
