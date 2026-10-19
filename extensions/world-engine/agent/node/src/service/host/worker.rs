//! The host-service worker: streaming offload sessions, staged jobs, and the
//! live-lane reconstructions whose generations fan out as world-model data.

use super::*;

/// One worker: claim a job under the lock, run the backend WITHOUT it, finalize
/// under the lock. Idles 500 ms when the queue is empty. A cancel that lands
/// during the backend run wins inside `finalize`. A real `file://` artifact under
/// the work root is rewritten to a fetchable LAN URL (the GCS reads it as the
/// output URL) before finalize, keeping the local path for any pipeline chaining.
pub(super) async fn worker_loop(
    state: Arc<Mutex<Engine>>,
    work_root: PathBuf,
    public_base: String,
    sessions: Arc<OffloadSessionManager>,
    world: Arc<WorldBroadcaster>,
    staged: world_engine_node::worker::WorkerCtx,
) {
    loop {
        let (prepared, reconstructor, detector) = {
            let engine = state.lock().await;
            let prepared = engine.scheduler().claim_and_prepare(now_ms());
            let (reconstructor, detector) = engine.scheduler().backends();
            (prepared, reconstructor, detector)
        };
        match prepared {
            // A session reconstruction, change detection or calibration runs
            // stage by stage on the shared staged worker.
            Ok(Prepared::Ready {
                job,
                input: PreparedInput::Staged(job_input),
            }) => world_engine_node::worker::run_staged_job(&staged, job, job_input).await,
            Ok(Prepared::Ready { job, input }) => {
                // A PerceptionOffload job carrying a `session` block requests a
                // continuous streaming session, not one-shot inference: start the
                // session (the node pulls the drone's RTSP feed -> detector ->
                // broadcaster -> the WS the drone subscribes to) and finalize the
                // job as accepted. The one-shot per-frame path below is unchanged.
                if let Some(spec) = SessionSpec::from_job_params(&job.params) {
                    let session_id = spec.id.clone();
                    sessions.start(spec).await;
                    let outcome = {
                        let engine = state.lock().await;
                        engine.scheduler().finalize(
                            &job,
                            BackendResult {
                                outputs: Vec::new(),
                                detections: Vec::new(),
                                error: None,
                            },
                            now_ms(),
                        )
                    };
                    match outcome {
                        Ok(o) => {
                            tracing::info!(job = %o.job_id, session = %session_id, "offload streaming session started")
                        }
                        Err(e) => {
                            tracing::error!(job = %job.id, error = %e, "offload session job finalize failed")
                        }
                    }
                    continue;
                }
                // Reconstruction is best-effort batch work and runs off this worker
                // on a blocking thread; a streaming perception-offload session is
                // latency-critical and runs on its OWN task, outside this batch
                // worker pool. They do not queue behind each other, so batch cannot
                // starve streaming of a worker slot. Before starting the heavy
                // reconstruction backend we still yield once when a session is live,
                // so any ready streaming work is polled first (WorkPriority::Streaming
                // > Batch). This is a cooperative nudge; fine-grained CPU/GPU
                // preemption of an in-flight reconstruction is out of scope.
                if matches!(input, PreparedInput::Reconstruct(_))
                    && sessions.should_yield_to_streaming()
                {
                    tokio::task::yield_now().await;
                }
                // Run the (real, possibly minutes-long) backend off the async
                // runtime thread so a long reconstruction never starves the HTTP
                // surface. The owned job + input move into the blocking task and
                // back out (PreparedInput is not Clone, and both are needed after
                // the run — input for the rerun/world-model write, job for finalize).
                let now = now_ms();
                let failed_job = job.clone();
                let joined = tokio::task::spawn_blocking(move || {
                    let r = Scheduler::run_backend(&*reconstructor, &*detector, &job, &input, now);
                    (r, job, input)
                })
                .await;
                // A panicking backend fails its job and the worker keeps
                // claiming: letting the panic unwind this task would take the
                // slot down for the life of the daemon and strand the job in
                // Running until the next restart.
                let (mut result, job, input) = match joined {
                    Ok(done) => done,
                    Err(e) => {
                        tracing::error!(job = %failed_job.id, error = %e, "backend task panicked");
                        let engine = state.lock().await;
                        if let Err(e) = engine.scheduler().finalize(
                            &failed_job,
                            BackendResult {
                                outputs: Vec::new(),
                                detections: Vec::new(),
                                error: Some(format!("backend panicked: {e}")),
                            },
                            now_ms(),
                        ) {
                            tracing::error!(job = %failed_job.id, error = %e, "finalize failed");
                        }
                        continue;
                    }
                };
                // Write the Rerun world-model .rrd from the real capture + the
                // reconstruction geometry so the GCS World viewer renders real data
                // (camera trajectory + the reconstructed point cloud). Reconstruct
                // jobs only; an offload job has no world model. Best-effort: a write
                // fault is logged and the job still completes with its other output.
                if let PreparedInput::Reconstruct(dataset) = &input {
                    let input_path = dataset
                        .meta
                        .get("input_path")
                        .and_then(|v| v.as_str())
                        .map(std::path::Path::new);
                    let geometry = result
                        .outputs
                        .first()
                        .map(|o| (o.kind.as_str(), o.uri.as_str()));
                    match build_rerun_output(&work_root, &job.id, input_path, geometry, now_ms()) {
                        Ok(Some(rerun_out)) => result.outputs.push(rerun_out),
                        Ok(None) => {}
                        Err(e) => {
                            tracing::warn!(job = %job.id, error = %e, "rerun world-model write failed")
                        }
                    }
                }
                for output in &mut result.outputs {
                    rewrite_output_to_artifact_url(output, &work_root, &public_base);
                }
                // Publish the generation as shared world-model DATA before
                // finalize consumes the result. This is the step that turns Atlas
                // from a private pipeline ending in a viewer into a consumable
                // data product: a plugin, a planner, or the GCS reads the
                // descriptors and pulls only the artifact it needs.
                // The reconstruction sits in the frame of its keyframe poses: the
                // ENU frame at the dataset's anchor, or a local frame without one.
                let placement = ados_capture_contract::Placement::identity(match &input {
                    PreparedInput::Reconstruct(dataset) => dataset
                        .meta
                        .get("input_path")
                        .and_then(|v| v.as_str())
                        .and_then(|p| world_engine_node::dataset_anchor(std::path::Path::new(p))),
                    _ => None,
                });
                publish_world_generation(
                    &world,
                    &job,
                    &result.outputs,
                    &work_root,
                    &public_base,
                    &placement,
                )
                .await;
                let outcome = {
                    let engine = state.lock().await;
                    engine.scheduler().finalize(&job, result, now_ms())
                };
                match outcome {
                    Ok(o) => tracing::info!(job = %o.job_id, state = ?o.state, "ran job"),
                    Err(e) => tracing::error!(job = %job.id, error = %e, "finalize failed"),
                }
            }
            Ok(Prepared::Failed(o)) => {
                tracing::info!(job = %o.job_id, state = ?o.state, "job failed at prepare");
            }
            Ok(Prepared::Empty) => tokio::time::sleep(Duration::from_millis(500)).await,
            Err(e) => {
                tracing::error!(error = %e, "worker claim failed");
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

/// Publish one completed reconstruction generation onto the world-model
/// descriptor stream, tagged with the drone that captured it.
///
/// Derived facts only: a gaussian count comes from the backend's own metadata or
/// from parsing the real `.ply`, bounds come from the real points, and the ESDF
/// is computed from the real geometry — a generation with nothing readable
/// publishes nothing rather than an empty world. Best-effort throughout: a
/// publish reaching zero subscribers is normal (no consumer connected), and a
/// derivation fault is logged without touching the job's own outcome.
pub(super) async fn publish_world_generation(
    world: &Arc<WorldBroadcaster>,
    job: &world_engine_node::JobRecord,
    outputs: &[world_engine_node::Output],
    work_root: &std::path::Path,
    public_base: &str,
    placement: &ados_capture_contract::Placement,
) {
    // A reconstruct job always carries its capture session; without one there is
    // nothing to attribute a world model to.
    let Some(session_id) = job.params.get("session_id").and_then(|v| v.as_str()) else {
        return;
    };
    // The drone the descriptors belong to. Absent on a pre-attribution capture,
    // and the per-device stream cannot route without it, so skip rather than
    // publish a world model under an empty device id.
    let Some(device_id) = job.params.get("device_id").and_then(|v| v.as_str()) else {
        tracing::debug!(job = %job.id, "world-model generation has no device attribution; not published");
        return;
    };
    let generation = job
        .params
        .get("generation")
        .and_then(|v| v.as_u64())
        .and_then(|g| u32::try_from(g).ok())
        .unwrap_or(0);

    let mut set: WorldDescriptorSet =
        derive_descriptors(session_id, generation, placement, outputs, work_root);
    // The ESDF is a real distance transform over the reconstruction, so it runs
    // off the async runtime like the backend does.
    let (sid, outs, wr, jid, pb, pl) = (
        session_id.to_string(),
        outputs.to_vec(),
        work_root.to_path_buf(),
        job.id.clone(),
        public_base.to_string(),
        placement.clone(),
    );
    match tokio::task::spawn_blocking(move || {
        derive_occupancy(&sid, generation, &pl, &outs, &wr, &jid, &pb)
    })
    .await
    {
        Ok(Ok(Some((desc, grid)))) => {
            tracing::info!(
                job = %job.id,
                generation,
                dims = ?desc.dims,
                resolution_m = desc.resolution_m,
                voxels = grid.distances.len(),
                "atlas esdf derived"
            );
            set.occupancy = Some(desc);
        }
        Ok(Ok(None)) => {}
        Ok(Err(e)) => tracing::warn!(job = %job.id, error = %e, "atlas_esdf_derive_failed"),
        Err(e) => tracing::warn!(job = %job.id, error = %e, "atlas_esdf_task_panicked"),
    }

    if set.is_empty() {
        return;
    }
    let mut published = 0usize;
    for (topic, payload) in [
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_SPLAT_TOPIC,
            set.splat.as_ref().map(|d| d.to_msgpack()),
        ),
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_POINTCLOUD_TOPIC,
            set.pointcloud.as_ref().map(|d| d.to_msgpack()),
        ),
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_MESH_TOPIC,
            set.mesh.as_ref().map(|d| d.to_msgpack()),
        ),
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_OCCUPANCY_TOPIC,
            set.occupancy.as_ref().map(|d| d.to_msgpack()),
        ),
    ] {
        match payload {
            Some(Ok(body)) => {
                world.publish(
                    device_id,
                    AtlasEvent::new(topic, Some(device_id.to_string()), body),
                );
                published += 1;
            }
            Some(Err(e)) => {
                tracing::warn!(topic, error = %e, "world_descriptor_encode_failed")
            }
            None => {}
        }
    }
    tracing::info!(
        job = %job.id,
        device = %device_id,
        session = %session_id,
        generation,
        descriptors = published,
        subscribers = world.subscriber_count(),
        "world-model generation published as shared data"
    );
}
