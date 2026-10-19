//! The session recorder: one writer thread owning a
//! [`ados_capture_contract::SessionWriter`] for the open session.
//!
//! The capture loop converts everything it records to session time and hands
//! it over as [`Item`]s through a bounded queue ([`QUEUE`]); a full queue
//! drops the item and counts it (per camera for images), so a slow disk never
//! stalls the loop. Images are encoded on the writer thread.
//!
//! Directory contract with the agent's storage janitor: `.recording` is
//! created right after `rig.json` (before the first segment byte), its mtime
//! is refreshed every [`TOUCH_EVERY`] while the session is open (recording or
//! paused), and it is removed only after `manifest.json` and `checksums.b3`
//! are written. The capture-active marker the FC router watches exists for
//! the same span.

pub mod convert;

use std::collections::HashMap;
use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use ados_capture_contract::foxglove::PoseInFrame;
use ados_capture_contract::foxglove::{LocationFix, PointCloud};
use ados_capture_contract::v1::{ClockSample, GnssRaw, ImuBatch, PoseCov, TriggerRecord};
use ados_capture_contract::{
    topics, Manifest, Operator, Rig, SegmentLimits, SessionHealth, SessionWriter,
};
use ados_protocol::framebus::FrameFormat;
use parking_lot::Mutex;

use convert::{ImageEncoding, ImageMsg};

/// Items the queue holds before new ones are dropped. Sensor records are
/// small and must survive an image backlog, so images have their own bound.
pub const QUEUE: usize = 4096;
/// Images that may wait for the writer before a new one is dropped.
pub const IMAGE_QUEUE: usize = 64;
/// How often `.recording` is refreshed while the session is open.
pub const TOUCH_EVERY: Duration = Duration::from_secs(30);
/// How often the recorded byte count is re-measured.
const MEASURE_EVERY: Duration = Duration::from_secs(2);

/// One thing to record, already on the session timebase.
#[derive(Debug)]
pub enum Item {
    Image {
        camera: String,
        t_ns: u64,
        format: FrameFormat,
        width: u32,
        height: u32,
        pixels: Vec<u8>,
    },
    Imu {
        src: String,
        t_ns: u64,
        batch: ImuBatch,
    },
    Trigger {
        camera: String,
        rec: TriggerRecord,
    },
    GnssRaw {
        src: String,
        raw: GnssRaw,
    },
    GnssFix {
        src: String,
        t_ns: u64,
        fix: LocationFix,
    },
    Lidar {
        src: String,
        t_ns: u64,
        cloud: PointCloud,
    },
    Pose {
        source: &'static str,
        t_ns: u64,
        pose: PoseInFrame,
        cov: PoseCov,
    },
    /// A bus clock sample as published, logged at `t_ns` (the session time of
    /// its host instant).
    Clock {
        t_ns: u64,
        sample: ClockSample,
    },
    Health {
        t_ns: u64,
        json: serde_json::Value,
    },
    /// A keyframe preview, written to `keyframes/<file>`.
    Keyframe {
        file: String,
        jpeg: Vec<u8>,
    },
    Finish {
        operator: Operator,
        health: SessionHealth,
        reply: mpsc::Sender<Result<Manifest, String>>,
    },
}

/// What the writer has done so far.
#[derive(Debug, Default, Clone)]
pub struct RecorderStats {
    pub images: HashMap<String, u64>,
    pub dropped: HashMap<String, u64>,
    pub dropped_other: u64,
    /// Items skipped because they could not be recorded (short buffer,
    /// encode failure, bad keyframe name), per kind (`image`, `keyframe`).
    pub bad_items: HashMap<&'static str, u64>,
    /// Items not written because the session writer had already failed.
    pub lost_after_error: u64,
    pub bytes: u64,
    /// Bytes per second over the last measurement.
    pub write_bps: f64,
    /// The first session-writer I/O error; the session stops recording once
    /// set.
    pub error: Option<String>,
}

/// Why one item was not written.
#[derive(Debug)]
enum WriteFail {
    /// The item itself is unusable; skip it and carry on.
    Bad(&'static str, anyhow::Error),
    /// The session writer failed; the recording cannot continue.
    Io(anyhow::Error),
}

impl From<ados_capture_contract::Error> for WriteFail {
    fn from(e: ados_capture_contract::Error) -> Self {
        Self::Io(e.into())
    }
}

/// Start parameters.
pub struct RecorderConfig {
    /// The local capture root (the sandbox mount of the host root).
    pub root: PathBuf,
    pub manifest: Manifest,
    pub rig: Rig,
    pub limits: SegmentLimits,
    pub encoding: ImageEncoding,
    /// The capture-active marker, created for the life of the session.
    pub active_marker: Option<PathBuf>,
}

/// Handle to the open session.
pub struct Recorder {
    tx: SyncSender<Item>,
    stats: Arc<Mutex<RecorderStats>>,
    pending_images: Arc<AtomicUsize>,
    thread: Option<JoinHandle<()>>,
    dir: PathBuf,
    marker: Option<PathBuf>,
}

impl Recorder {
    /// Create the session directory, `.recording`, and the writer thread.
    pub fn start(cfg: RecorderConfig) -> anyhow::Result<Self> {
        let writer =
            SessionWriter::create(&cfg.root, cfg.manifest, &cfg.rig)?.with_limits(cfg.limits);
        let dir = writer.dir().to_path_buf();
        File::create(dir.join(crate::storage::RECORDING_MARKER))?;
        fs::create_dir_all(dir.join("keyframes"))?;
        let stats = Arc::new(Mutex::new(RecorderStats::default()));
        let (tx, rx) = mpsc::sync_channel(QUEUE);
        let thread_stats = stats.clone();
        let thread_dir = dir.clone();
        let encoding = cfg.encoding;
        let pending_images = Arc::new(AtomicUsize::new(0));
        let thread_pending = pending_images.clone();
        let thread = std::thread::Builder::new()
            .name("capture-recorder".into())
            .spawn(move || {
                write_loop(
                    writer,
                    rx,
                    thread_stats,
                    thread_dir,
                    encoding,
                    thread_pending,
                )
            })?;
        // The capture-active marker comes last so no failure above leaves
        // it behind.
        if let Some(m) = &cfg.active_marker {
            if let Err(e) = File::create(m) {
                tracing::warn!(path = %m.display(), error = %e, "capture_active_marker_failed");
            }
        }
        Ok(Self {
            tx,
            stats,
            pending_images,
            thread: Some(thread),
            dir,
            marker: cfg.active_marker,
        })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    pub fn session_id(&self) -> String {
        self.dir
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default()
    }

    /// Queue an item; `false` when it was dropped (queue full or writer gone).
    pub fn submit(&self, item: Item) -> bool {
        let camera = match &item {
            Item::Image { camera, .. } => Some(camera.clone()),
            _ => None,
        };
        let full = camera.is_some() && self.pending_images.load(Ordering::Relaxed) >= IMAGE_QUEUE;
        let sent = if full {
            Err(TrySendError::Full(item))
        } else {
            self.tx.try_send(item)
        };
        match sent {
            Ok(()) => {
                if camera.is_some() {
                    self.pending_images.fetch_add(1, Ordering::Relaxed);
                }
                true
            }
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                let mut s = self.stats.lock();
                match camera {
                    Some(c) => *s.dropped.entry(c).or_default() += 1,
                    None => s.dropped_other += 1,
                }
                false
            }
        }
    }

    /// Queue an item, waiting for room (replay and tests, where nothing is
    /// live); `false` when the writer is gone.
    pub fn send(&self, item: Item) -> bool {
        let image = matches!(item, Item::Image { .. });
        let ok = self.tx.send(item).is_ok();
        if ok && image {
            self.pending_images.fetch_add(1, Ordering::Relaxed);
        }
        ok
    }

    pub fn stats(&self) -> RecorderStats {
        self.stats.lock().clone()
    }

    /// Close the session: flush the queue, write `manifest.json` and
    /// `checksums.b3`, then remove `.recording` and the active marker.
    pub fn finish(mut self, operator: Operator, health: SessionHealth) -> anyhow::Result<Manifest> {
        let (reply, rx) = mpsc::channel();
        let sent = self.tx.send(Item::Finish {
            operator,
            health,
            reply,
        });
        let result = match sent {
            Ok(()) => rx
                .recv()
                .map_err(|_| anyhow::anyhow!("recorder thread ended"))?
                .map_err(|e| anyhow::anyhow!(e)),
            Err(_) => Err(anyhow::anyhow!("recorder thread ended")),
        };
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        if result.is_ok() {
            let _ = fs::remove_file(self.dir.join(crate::storage::RECORDING_MARKER));
        }
        if let Some(m) = self.marker.take() {
            let _ = fs::remove_file(m);
        }
        result
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        // An unfinished session keeps `.recording` (the janitor sees a crashed
        // session) but must not leave the FC at capture rates.
        if let Some(m) = self.marker.take() {
            let _ = fs::remove_file(m);
        }
    }
}

fn write_loop(
    mut w: SessionWriter,
    rx: mpsc::Receiver<Item>,
    stats: Arc<Mutex<RecorderStats>>,
    dir: PathBuf,
    encoding: ImageEncoding,
    pending_images: Arc<AtomicUsize>,
) {
    let marker = dir.join(crate::storage::RECORDING_MARKER);
    let mut touched = Instant::now();
    let mut measured = (Instant::now(), 0u64);
    loop {
        let item = match rx.recv_timeout(Duration::from_secs(1)) {
            Ok(item) => Some(item),
            Err(RecvTimeoutError::Timeout) => None,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        if touched.elapsed() >= TOUCH_EVERY {
            touch(&marker);
            touched = Instant::now();
        }
        if measured.0.elapsed() >= MEASURE_EVERY {
            let bytes = dir_bytes(&dir.join("segments"));
            let dt = measured.0.elapsed().as_secs_f64();
            let mut s = stats.lock();
            s.write_bps = bytes.saturating_sub(measured.1) as f64 / dt;
            s.bytes = bytes;
            measured = (Instant::now(), bytes);
        }
        let Some(item) = item else { continue };
        if let Item::Finish {
            operator,
            health,
            reply,
        } = item
        {
            let m = w.manifest_mut();
            m.operator = operator;
            m.health = health;
            let _ = reply.send(w.finish().map_err(|e| e.to_string()));
            return;
        }
        if matches!(item, Item::Image { .. }) {
            pending_images.fetch_sub(1, Ordering::Relaxed);
        }
        if stats.lock().error.is_some() {
            stats.lock().lost_after_error += 1;
            continue;
        }
        match write_item(&mut w, &dir, item, encoding, &stats) {
            Ok(()) => {}
            Err(WriteFail::Bad(kind, e)) => {
                tracing::warn!(kind, error = %e, "capture_record_item_skipped");
                *stats.lock().bad_items.entry(kind).or_default() += 1;
            }
            Err(WriteFail::Io(e)) => {
                tracing::error!(error = %e, "capture_record_failed");
                stats.lock().error = Some(e.to_string());
            }
        }
    }
}

fn write_item(
    w: &mut SessionWriter,
    dir: &Path,
    item: Item,
    encoding: ImageEncoding,
    stats: &Mutex<RecorderStats>,
) -> Result<(), WriteFail> {
    match item {
        Item::Image {
            camera,
            t_ns,
            format,
            width,
            height,
            pixels,
        } => {
            let topic = topics::cam_image(&camera);
            let msg = convert::image(&camera, t_ns, format, width, height, pixels, encoding)
                .map_err(|e| WriteFail::Bad("image", e))?;
            match msg {
                ImageMsg::Raw(m) => w.write_proto(&topic, t_ns, &m)?,
                ImageMsg::Compressed(m) => w.write_proto(&topic, t_ns, &m)?,
            }
            *stats.lock().images.entry(camera).or_default() += 1;
        }
        Item::Imu { src, t_ns, batch } => w.write_proto(&topics::imu(&src), t_ns, &batch)?,
        Item::Trigger { camera, rec } => {
            w.write_proto(&topics::cam_trigger(&camera), rec.t_mid_ns, &rec)?
        }
        Item::GnssRaw { src, raw } => w.write_proto(&topics::gnss_raw(&src), raw.t_ns, &raw)?,
        Item::GnssFix { src, t_ns, fix } => w.write_proto(&topics::gnss_fix(&src), t_ns, &fix)?,
        Item::Lidar { src, t_ns, cloud } => {
            w.write_proto(&topics::lidar_points(&src), t_ns, &cloud)?
        }
        Item::Pose {
            source,
            t_ns,
            pose,
            cov,
        } => {
            w.write_proto(&topics::pose(source), t_ns, &pose)?;
            w.write_proto(&topics::pose_cov(source), t_ns, &cov)?;
        }
        Item::Clock { t_ns, sample } => w.write_proto(topics::CLOCK, t_ns, &sample)?,
        Item::Health { t_ns, json } => w.write_json(topics::HEALTH, t_ns, &json)?,
        Item::Keyframe { file, jpeg } => {
            if !crate::storage::safe_session_id(file.trim_end_matches(".jpg")) {
                return Err(WriteFail::Bad(
                    "keyframe",
                    anyhow::anyhow!("bad keyframe file name `{file}`"),
                ));
            }
            // A keyframe preview is outside the session log; a failed write
            // loses that preview only.
            fs::write(dir.join("keyframes").join(file), jpeg)
                .map_err(|e| WriteFail::Bad("keyframe", e.into()))?;
        }
        Item::Finish { .. } => unreachable!("handled by the loop"),
    }
    Ok(())
}

fn touch(path: &Path) {
    let r = File::options()
        .write(true)
        .open(path)
        .and_then(|f| f.set_modified(SystemTime::now()));
    if let Err(e) = r {
        tracing::warn!(path = %path.display(), error = %e, "recording_marker_touch_failed");
    }
}

fn dir_bytes(dir: &Path) -> u64 {
    fs::read_dir(dir)
        .map(|rd| {
            rd.filter_map(Result::ok)
                .filter_map(|e| e.metadata().ok())
                .map(|m| m.len())
                .sum()
        })
        .unwrap_or(0)
}

#[cfg(test)]
pub(crate) mod tests;
