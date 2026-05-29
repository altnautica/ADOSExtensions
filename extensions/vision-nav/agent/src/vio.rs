//! VIO vendor-binary bridge.
//!
//! A [`VioEngine`] owns one spawned vendor binary (OpenVINS or
//! VINS-Fusion) and the two IPC channels it speaks:
//!
//! * a POSIX shared-memory ring for camera frames (the binary opens it
//!   read-only with `shm_open` and reads the highest-sequence slot),
//!   and
//! * a length-prefixed msgpack control channel over a Unix-domain
//!   socket the plugin listens on and the binary connects to
//!   (`hello`/`config`/`imu`/`frame_ready` out; `hello_ack`/`pose`/
//!   `alive`/`log` in).
//!
//! The Rust plugin no longer captures frames; it bridges the shared
//! vision-bus frames into the SHM ring this engine owns, then notifies
//! the binary with a `frame_ready` control message.
//!
//! Both channels' byte layouts match the vendored C++ adapters
//! (`vendor/openvins`, `vendor/vins-fusion`) exactly so the binaries
//! run unchanged. The SHM slot layout matches `shm_ring.cpp`:
//! 8 slots, each `FrameSlotHeader` then up to 4 MiB of pixels.
//!
//! Safety note: a VIO engine that fails to start, loses its heartbeat,
//! or returns a malformed pose must NOT silently feed a wrong pose into
//! the EKF. [`VioEngine::start`] returns an error (the estimator then
//! falls back to a safe no-emit state); a missed heartbeat tears the
//! engine down; a pose that fails to decode is dropped.

use std::io::{Read, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::Serialize;

/// SHM frame-format tags shared with the C++ side.
pub const FRAME_FORMAT_GRAY8: u32 = 0;
pub const FRAME_FORMAT_NV12: u32 = 1;
pub const FRAME_FORMAT_RGB8: u32 = 2;

/// Control-channel protocol version. Must match the C++ shim.
const PROTOCOL_VERSION: u32 = 1;

/// SHM ring geometry, matching `vendor/.../shm_ring.cpp`.
const SLOT_COUNT: usize = 8;
const MAX_PAYLOAD_BYTES: usize = 4 * 1024 * 1024;
/// `FrameSlotHeader` is `u64 + 4*u32 + u64 + u32` laid out by the C++
/// compiler with default alignment: the trailing `u32 payload_size`
/// sits after a `u64`, so the struct is padded to an 8-byte multiple.
/// Lay the header out explicitly to match: 8 + 4 + 4 + 4 + 4 + 8 + 4 = 36,
/// padded to 40 for 8-byte alignment of the next slot's leading u64.
const FRAME_SLOT_HEADER_LEN: usize = 40;
const SLOT_SIZE: usize = FRAME_SLOT_HEADER_LEN + MAX_PAYLOAD_BYTES;

/// 4 MiB control-frame cap, matching the Python `MAX_MESSAGE_BYTES`.
const MAX_MESSAGE_BYTES: usize = 4 * 1024 * 1024;

/// Default heartbeat grace before the watchdog tears the engine down.
pub const DEFAULT_HEARTBEAT_GRACE: Duration = Duration::from_secs(2);

/// Per-session config sent to the vendor on connect. Mirrors the
/// Python `EngineConfig` + its `_config_to_wire` shape.
#[derive(Debug, Clone)]
pub struct EngineConfig {
    pub camera_model: String,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub width: u32,
    pub height: u32,
    pub distortion_model: String,
    pub distortion_coeffs: Vec<f64>,
    /// 16 row-major floats (T_cam_imu).
    pub t_cam_imu: Vec<f64>,
    pub timeshift_cam_imu_s: f64,
    pub imu_rate_hz: f64,
    pub camera_rate_hz: f64,
}

/// One pose return from the vendor binary.
#[derive(Debug, Clone, PartialEq)]
pub struct PoseMessage {
    pub ts_us: u64,
    pub position: (f32, f32, f32),
    pub orientation_quat: (f32, f32, f32, f32), // (w, x, y, z)
    pub velocity: (f32, f32, f32),
    pub covariance: Vec<f32>,
    pub feature_count: i32,
    pub reset_counter: u32,
    pub state: String,
}

/// Errors raised starting / running an engine.
#[derive(Debug)]
pub enum VioError {
    Io(std::io::Error),
    Handshake(String),
    NotStarted,
}

impl std::fmt::Display for VioError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VioError::Io(e) => write!(f, "io: {e}"),
            VioError::Handshake(s) => write!(f, "handshake: {s}"),
            VioError::NotStarted => write!(f, "engine not started"),
        }
    }
}
impl std::error::Error for VioError {}
impl From<std::io::Error> for VioError {
    fn from(e: std::io::Error) -> Self {
        VioError::Io(e)
    }
}

// ---------------------------------------------------------------------------
// msgpack control-channel message shapes.
// ---------------------------------------------------------------------------

/// An outbound `imu` control message.
#[derive(Serialize)]
struct ImuMsg {
    #[serde(rename = "type")]
    kind: &'static str,
    ts_us: u64,
    gyro: [f64; 3],
    accel: [f64; 3],
}

/// An outbound `frame_ready` control message.
#[derive(Serialize)]
struct FrameReadyMsg {
    #[serde(rename = "type")]
    kind: &'static str,
    ts_us: u64,
    seq: u64,
    slot: u32,
    width: u32,
    height: u32,
    stride: u32,
    format: u32,
}

/// A spawn authorization callback: given the vendor binary basename and
/// argv tail, returns whether the host authorized the spawn. The plugin
/// performs the actual exec (so the child inherits the runner's cgroup
/// slice); the host enforces the manifest allowlist.
pub type SpawnAuthorizer<'a> =
    Box<dyn Fn(&str, &[String]) -> Result<(), VioError> + Send + 'a>;

/// One VIO engine. Owns the listening UDS, the SHM ring, the spawned
/// vendor child, and the pose queue.
pub struct VioEngine {
    engine_id: &'static str,
    basename: &'static str,
    socket_path: PathBuf,
    shm_name: String,
    ring: Option<FrameRing>,
    /// The accepted control connection to the vendor binary.
    conn: Option<UnixStream>,
    recv_buf: Vec<u8>,
    poses: Arc<Mutex<Vec<PoseMessage>>>,
    last_alive: Arc<AtomicU64>, // monotonic ns of last `alive`
    heartbeat_grace: Duration,
    child: Option<std::process::Child>,
    started: bool,
}

impl VioEngine {
    /// Construct an OpenVINS-flavoured engine.
    pub fn openvins(socket_path: impl Into<PathBuf>, shm_name: impl Into<String>) -> Self {
        Self::new("openvins", "ados_openvins_shim", socket_path, shm_name)
    }

    /// Construct a VINS-Fusion-flavoured engine.
    pub fn vins_fusion(socket_path: impl Into<PathBuf>, shm_name: impl Into<String>) -> Self {
        Self::new(
            "vins-fusion",
            "ados_vins_fusion_shim",
            socket_path,
            shm_name,
        )
    }

    fn new(
        engine_id: &'static str,
        basename: &'static str,
        socket_path: impl Into<PathBuf>,
        shm_name: impl Into<String>,
    ) -> Self {
        Self {
            engine_id,
            basename,
            socket_path: socket_path.into(),
            shm_name: shm_name.into(),
            ring: None,
            conn: None,
            recv_buf: Vec::new(),
            poses: Arc::new(Mutex::new(Vec::new())),
            last_alive: Arc::new(AtomicU64::new(0)),
            heartbeat_grace: DEFAULT_HEARTBEAT_GRACE,
            child: None,
            started: false,
        }
    }

    pub fn engine_id(&self) -> &'static str {
        self.engine_id
    }

    pub fn basename(&self) -> &'static str {
        self.basename
    }

    pub fn started(&self) -> bool {
        self.started
    }

    /// Spawn the vendor binary and complete the handshake.
    ///
    /// `install_dir` is where the host placed the plugin tree; the
    /// vendor binary lives at `<install_dir>/vendor/<basename>`. `authorize`
    /// asks the host to clear the spawn against the manifest allowlist
    /// before the plugin execs the child. The listener is bound first so
    /// the binary can connect immediately on start.
    pub fn start(
        &mut self,
        config: &EngineConfig,
        install_dir: &str,
        authorize: SpawnAuthorizer<'_>,
    ) -> Result<(), VioError> {
        if self.started {
            return Ok(());
        }

        // Create the SHM ring the binary will mmap.
        let ring = FrameRing::create(&self.shm_name)?;
        self.ring = Some(ring);

        // Bind the control socket before spawning so the connect races
        // cleanly.
        let _ = std::fs::remove_file(&self.socket_path);
        let listener = UnixListener::bind(&self.socket_path)?;

        // Authorize + spawn the vendor binary.
        let args = self.spawn_args();
        authorize(self.basename, &args)?;
        let bin_path = PathBuf::from(install_dir).join("vendor").join(self.basename);
        let child = std::process::Command::new(&bin_path).args(&args).spawn()?;
        self.child = Some(child);

        // Accept the binary's connection (it connects on start).
        listener.set_nonblocking(false)?;
        let (conn, _addr) = listener.accept()?;
        conn.set_read_timeout(Some(Duration::from_millis(50)))?;
        self.conn = Some(conn);

        // Handshake: hello -> hello_ack, then config.
        self.handshake(config)?;
        self.started = true;
        Ok(())
    }

    /// The argv tail passed to the vendor binary (matches the C++
    /// `--socket`/`--shm`/`--slot-count`/`--slot-size` flags).
    fn spawn_args(&self) -> Vec<String> {
        vec![
            "--socket".into(),
            self.socket_path.to_string_lossy().into_owned(),
            "--shm".into(),
            self.shm_name.clone(),
            "--slot-count".into(),
            SLOT_COUNT.to_string(),
            "--slot-size".into(),
            SLOT_SIZE.to_string(),
        ]
    }

    fn handshake(&mut self, config: &EngineConfig) -> Result<(), VioError> {
        self.send_msg(&rmpv::Value::Map(vec![
            (rmpv::Value::from("type"), rmpv::Value::from("hello")),
            (
                rmpv::Value::from("protocol_version"),
                rmpv::Value::from(PROTOCOL_VERSION),
            ),
            (rmpv::Value::from("engine"), rmpv::Value::from(self.engine_id)),
        ]))?;

        // Block (within the read timeout, retried) for the hello_ack.
        let ack = self.recv_msg_blocking(Duration::from_secs(5))?;
        let kind = map_str(&ack, "type");
        if kind.as_deref() != Some("hello_ack") {
            return Err(VioError::Handshake(format!("unexpected reply: {kind:?}")));
        }
        let ver = map_u64(&ack, "protocol_version");
        if ver != Some(PROTOCOL_VERSION as u64) {
            return Err(VioError::Handshake(format!(
                "protocol mismatch: plugin={PROTOCOL_VERSION} vendor={ver:?}"
            )));
        }
        self.send_msg(&config_to_wire(config))?;
        // The vendor's first alive after config opens the watchdog grace.
        self.last_alive.store(now_ns() as u64, Ordering::Relaxed);
        Ok(())
    }

    /// Stop: send shutdown, close the channel, terminate the binary.
    pub fn stop(&mut self) {
        if !self.started {
            return;
        }
        let _ = self.send_msg(&rmpv::Value::Map(vec![(
            rmpv::Value::from("type"),
            rmpv::Value::from("shutdown"),
        )]));
        self.conn = None;
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
        self.ring = None;
        let _ = std::fs::remove_file(&self.socket_path);
        self.started = false;
    }

    /// Forward one IMU sample to the binary.
    pub fn send_imu(
        &mut self,
        ts_us: u64,
        gyro: (f32, f32, f32),
        accel: (f32, f32, f32),
    ) -> Result<(), VioError> {
        if !self.started {
            return Err(VioError::NotStarted);
        }
        let msg = ImuMsg {
            kind: "imu",
            ts_us,
            gyro: [gyro.0 as f64, gyro.1 as f64, gyro.2 as f64],
            accel: [accel.0 as f64, accel.1 as f64, accel.2 as f64],
        };
        let bytes = rmp_serde::to_vec_named(&msg)
            .map_err(|e| VioError::Handshake(format!("imu encode: {e}")))?;
        self.send_raw(&bytes)
    }

    /// Push a frame into the SHM ring and notify the binary.
    #[allow(clippy::too_many_arguments)]
    pub fn send_frame(
        &mut self,
        ts_us: u64,
        width: u32,
        height: u32,
        stride: u32,
        pixel_format: u32,
        pixels: &[u8],
    ) -> Result<(), VioError> {
        if !self.started {
            return Err(VioError::NotStarted);
        }
        let ring = self.ring.as_mut().ok_or(VioError::NotStarted)?;
        let slot = ring.write(ts_us, width, height, stride, pixel_format, pixels)?;
        let msg = FrameReadyMsg {
            kind: "frame_ready",
            ts_us: slot.ts_us,
            seq: slot.seq,
            slot: slot.slot_index,
            width: slot.width,
            height: slot.height,
            stride: slot.stride,
            format: slot.pixel_format,
        };
        let bytes = rmp_serde::to_vec_named(&msg)
            .map_err(|e| VioError::Handshake(format!("frame_ready encode: {e}")))?;
        self.send_raw(&bytes)
    }

    /// Drain any pending control messages. Pose messages land on the
    /// queue; `alive` refreshes the watchdog; `log` is forwarded to
    /// stderr. Non-blocking: returns when the socket has no more bytes.
    pub fn poll(&mut self) {
        if !self.started {
            return;
        }
        loop {
            match self.try_recv_msg() {
                Ok(Some(msg)) => self.dispatch(msg),
                Ok(None) => return, // would block / no full frame yet
                Err(_) => return,   // peer closed; the watchdog handles restart
            }
        }
    }

    /// Return and clear the accumulated poses.
    pub fn drain_poses(&self) -> Vec<PoseMessage> {
        let mut g = self.poses.lock().expect("pose lock");
        std::mem::take(&mut *g)
    }

    /// False when no `alive` arrived within the grace window. True
    /// before the first alive (a slow-starting binary is not aborted).
    pub fn is_alive(&self, now_ns: i64) -> bool {
        let last = self.last_alive.load(Ordering::Relaxed);
        if last == 0 {
            return true;
        }
        ((now_ns as u64).saturating_sub(last)) as i64 <= self.heartbeat_grace.as_nanos() as i64
    }

    fn dispatch(&self, msg: rmpv::Value) {
        match map_str(&msg, "type").as_deref() {
            Some("pose") => {
                if let Some(pose) = decode_pose(&msg) {
                    self.poses.lock().expect("pose lock").push(pose);
                } else {
                    eprintln!("vision-nav: vio pose decode failed");
                }
            }
            Some("alive") => {
                self.last_alive.store(now_ns() as u64, Ordering::Relaxed);
            }
            Some("log") => {
                let text = map_str(&msg, "msg").unwrap_or_default();
                eprintln!("vision-nav: vio engine: {text}");
            }
            _ => {}
        }
    }

    // ---- framing over the UDS control channel ------------------------

    fn send_msg(&mut self, value: &rmpv::Value) -> Result<(), VioError> {
        let body = rmp_serde::to_vec_named(value)
            .map_err(|e| VioError::Handshake(format!("encode: {e}")))?;
        self.send_raw(&body)
    }

    fn send_raw(&mut self, body: &[u8]) -> Result<(), VioError> {
        if body.len() > MAX_MESSAGE_BYTES {
            return Err(VioError::Handshake("message too large".into()));
        }
        let conn = self.conn.as_mut().ok_or(VioError::NotStarted)?;
        let header = (body.len() as u32).to_be_bytes();
        conn.write_all(&header)?;
        conn.write_all(body)?;
        conn.flush()?;
        Ok(())
    }

    /// Try to read one complete framed msgpack message without blocking
    /// beyond the socket read timeout. `Ok(None)` when no full frame is
    /// buffered yet.
    fn try_recv_msg(&mut self) -> Result<Option<rmpv::Value>, VioError> {
        // Pull whatever is available into the buffer.
        let conn = self.conn.as_mut().ok_or(VioError::NotStarted)?;
        let mut chunk = [0u8; 8192];
        loop {
            match conn.read(&mut chunk) {
                Ok(0) => break, // EOF
                Ok(n) => self.recv_buf.extend_from_slice(&chunk[..n]),
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    break
                }
                Err(e) => return Err(VioError::Io(e)),
            }
            if self.recv_buf.len() > MAX_MESSAGE_BYTES * 2 {
                break;
            }
        }
        self.take_framed()
    }

    /// Block (re-reading within the timeout) for one full framed
    /// message. Used only for the handshake.
    fn recv_msg_blocking(&mut self, timeout: Duration) -> Result<rmpv::Value, VioError> {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            if let Some(v) = self.take_framed()? {
                return Ok(v);
            }
            if std::time::Instant::now() >= deadline {
                return Err(VioError::Handshake("timed out awaiting reply".into()));
            }
            let conn = self.conn.as_mut().ok_or(VioError::NotStarted)?;
            let mut chunk = [0u8; 8192];
            match conn.read(&mut chunk) {
                Ok(0) => return Err(VioError::Handshake("peer closed".into())),
                Ok(n) => self.recv_buf.extend_from_slice(&chunk[..n]),
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(VioError::Io(e)),
            }
        }
    }

    /// Decode one length-prefixed msgpack frame out of `recv_buf` if a
    /// full one is present.
    fn take_framed(&mut self) -> Result<Option<rmpv::Value>, VioError> {
        if self.recv_buf.len() < 4 {
            return Ok(None);
        }
        let len = u32::from_be_bytes([
            self.recv_buf[0],
            self.recv_buf[1],
            self.recv_buf[2],
            self.recv_buf[3],
        ]) as usize;
        if len > MAX_MESSAGE_BYTES {
            return Err(VioError::Handshake("declared frame too large".into()));
        }
        if self.recv_buf.len() < 4 + len {
            return Ok(None);
        }
        let body = self.recv_buf[4..4 + len].to_vec();
        self.recv_buf.drain(0..4 + len);
        let value: rmpv::Value = rmp_serde::from_slice(&body)
            .map_err(|e| VioError::Handshake(format!("decode: {e}")))?;
        Ok(Some(value))
    }
}

impl Drop for VioEngine {
    fn drop(&mut self) {
        if self.started {
            self.stop();
        }
    }
}

// ---------------------------------------------------------------------------
// SHM frame ring (writer side). Mirrors the C++ reader's slot layout.
// ---------------------------------------------------------------------------

/// One written frame's slot descriptor.
struct FrameSlot {
    slot_index: u32,
    ts_us: u64,
    seq: u64,
    width: u32,
    height: u32,
    stride: u32,
    pixel_format: u32,
}

/// A `/dev/shm`-backed ring of fixed-size slots. The C++ binary opens
/// the same name read-only and reads the highest-sequence slot. We
/// write the slot header in the exact field order the C++
/// `FrameSlotHeader` declares.
struct FrameRing {
    name: String,
    path: PathBuf,
    map: memmap2::MmapMut,
    write_idx: usize,
    seq: u64,
}

impl FrameRing {
    fn create(name: &str) -> Result<Self, VioError> {
        // The C++ binary uses shm_open(name); glibc maps that to
        // /dev/shm/<name without leading slash>. Create a file there of
        // the full ring size.
        let bare = name.trim_start_matches('/');
        Self::create_at(name, &PathBuf::from(format!("/dev/shm/{bare}")))
    }

    /// Create the ring at an explicit path. Production passes a
    /// `/dev/shm/<name>` path (so the C++ `shm_open(name)` maps the same
    /// region); tests pass a tempdir path on hosts without `/dev/shm`.
    fn create_at(name: &str, path: &std::path::Path) -> Result<Self, VioError> {
        let path = path.to_path_buf();
        let total = SLOT_COUNT * SLOT_SIZE;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&path)?;
        file.set_len(total as u64)?;
        // SAFETY: we created and sized the file; the engine maps it
        // read-only. The single writer here owns the slot recycle.
        let map = unsafe { memmap2::MmapMut::map_mut(&file) }?;
        Ok(Self {
            name: name.to_string(),
            path,
            map,
            write_idx: 0,
            seq: 0,
        })
    }

    /// Copy one frame into the next slot and stamp its header.
    #[allow(clippy::too_many_arguments)]
    fn write(
        &mut self,
        ts_us: u64,
        width: u32,
        height: u32,
        stride: u32,
        pixel_format: u32,
        pixels: &[u8],
    ) -> Result<FrameSlot, VioError> {
        if pixels.len() > MAX_PAYLOAD_BYTES {
            return Err(VioError::Handshake(format!(
                "frame {} exceeds slot payload {MAX_PAYLOAD_BYTES}",
                pixels.len()
            )));
        }
        let slot_index = (self.write_idx % SLOT_COUNT) as u32;
        let base = slot_index as usize * SLOT_SIZE;
        let seq = self.seq;
        // Write the payload first, then the header sequence last so a
        // reader that races sees the full payload by the time the new
        // sequence is visible (the C++ reader keys on the sequence).
        let data_off = base + FRAME_SLOT_HEADER_LEN;
        self.map[data_off..data_off + pixels.len()].copy_from_slice(pixels);

        // FrameSlotHeader field order (see vendor shm_ring.hpp):
        //   u64 sequence; u32 width; u32 height; u32 stride; u32 format;
        //   u64 timestamp_ns; u32 payload_size;
        let ts_ns = ts_us.saturating_mul(1000);
        write_u64(&mut self.map, base, seq);
        write_u32(&mut self.map, base + 8, width);
        write_u32(&mut self.map, base + 12, height);
        write_u32(&mut self.map, base + 16, stride);
        write_u32(&mut self.map, base + 20, pixel_format);
        write_u64(&mut self.map, base + 24, ts_ns);
        write_u32(&mut self.map, base + 32, pixels.len() as u32);

        self.write_idx += 1;
        self.seq += 1;
        Ok(FrameSlot {
            slot_index,
            ts_us,
            seq,
            width,
            height,
            stride,
            pixel_format,
        })
    }
}

impl Drop for FrameRing {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
        let _ = &self.name;
    }
}

fn write_u32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}
fn write_u64(buf: &mut [u8], off: usize, v: u64) {
    buf[off..off + 8].copy_from_slice(&v.to_le_bytes());
}

// ---------------------------------------------------------------------------
// Wire helpers.
// ---------------------------------------------------------------------------

fn config_to_wire(c: &EngineConfig) -> rmpv::Value {
    use rmpv::Value;
    let model = format!("{}-{}", c.camera_model, c.distortion_model);
    let intrinsics = Value::Map(vec![
        (Value::from("model"), Value::from(model)),
        (Value::from("fx"), Value::from(c.fx)),
        (Value::from("fy"), Value::from(c.fy)),
        (Value::from("cx"), Value::from(c.cx)),
        (Value::from("cy"), Value::from(c.cy)),
        (Value::from("width"), Value::from(c.width)),
        (Value::from("height"), Value::from(c.height)),
        (
            Value::from("distortion_coeffs"),
            Value::Array(c.distortion_coeffs.iter().map(|&x| Value::from(x)).collect()),
        ),
    ]);
    let extrinsics = Value::Map(vec![(
        Value::from("T_cam_imu"),
        Value::Array(c.t_cam_imu.iter().map(|&x| Value::from(x)).collect()),
    )]);
    Value::Map(vec![
        (Value::from("type"), Value::from("config")),
        (Value::from("intrinsics"), intrinsics),
        (Value::from("extrinsics"), extrinsics),
        (
            Value::from("timeshift_cam_imu_s"),
            Value::from(c.timeshift_cam_imu_s),
        ),
        (Value::from("imu_rate_hz"), Value::from(c.imu_rate_hz)),
        (Value::from("camera_rate_hz"), Value::from(c.camera_rate_hz)),
    ])
}

fn decode_pose(msg: &rmpv::Value) -> Option<PoseMessage> {
    let state = map_str(msg, "state").unwrap_or_else(|| "init".into());
    if !matches!(
        state.as_str(),
        "init" | "converging" | "converged" | "degraded" | "failed"
    ) {
        return None;
    }
    let pos = map_f32_array(msg, "position")?;
    let quat = map_f32_array(msg, "orientation_quat")?;
    let vel = map_f32_array(msg, "velocity")?;
    if pos.len() != 3 || vel.len() != 3 || quat.len() != 4 {
        return None;
    }
    let cov = map_f32_array(msg, "covariance").unwrap_or_default();
    Some(PoseMessage {
        ts_us: map_u64(msg, "ts_us")?,
        position: (pos[0], pos[1], pos[2]),
        orientation_quat: (quat[0], quat[1], quat[2], quat[3]),
        velocity: (vel[0], vel[1], vel[2]),
        covariance: cov,
        feature_count: map_u64(msg, "feature_count").unwrap_or(0) as i32,
        reset_counter: map_u64(msg, "reset_counter").unwrap_or(0) as u32,
        state,
    })
}

fn map_get<'a>(v: &'a rmpv::Value, key: &str) -> Option<&'a rmpv::Value> {
    match v {
        rmpv::Value::Map(entries) => entries
            .iter()
            .find(|(k, _)| k.as_str() == Some(key))
            .map(|(_, val)| val),
        _ => None,
    }
}

fn map_str(v: &rmpv::Value, key: &str) -> Option<String> {
    map_get(v, key).and_then(|x| x.as_str()).map(str::to_string)
}

fn map_u64(v: &rmpv::Value, key: &str) -> Option<u64> {
    map_get(v, key).and_then(|x| x.as_u64().or_else(|| x.as_i64().map(|i| i as u64)))
}

fn map_f32_array(v: &rmpv::Value, key: &str) -> Option<Vec<f32>> {
    match map_get(v, key)? {
        rmpv::Value::Array(items) => {
            let mut out = Vec::with_capacity(items.len());
            for item in items {
                let f = item
                    .as_f64()
                    .or_else(|| item.as_i64().map(|i| i as f64))?;
                out.push(f as f32);
            }
            Some(out)
        }
        _ => None,
    }
}

fn now_ns() -> i64 {
    crate::mavlink_emit::monotonic_ns()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_to_wire_has_expected_shape() {
        let cfg = EngineConfig {
            camera_model: "pinhole".into(),
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
            distortion_model: "radtan".into(),
            distortion_coeffs: vec![0.1, 0.0, 0.0, 0.0],
            t_cam_imu: vec![1.0; 16],
            timeshift_cam_imu_s: -0.005,
            imu_rate_hz: 100.0,
            camera_rate_hz: 30.0,
        };
        let v = config_to_wire(&cfg);
        assert_eq!(map_str(&v, "type").as_deref(), Some("config"));
        let intr = map_get(&v, "intrinsics").unwrap();
        // model is "<camera_model>-<distortion_model>".
        assert_eq!(map_str(intr, "model").as_deref(), Some("pinhole-radtan"));
    }

    #[test]
    fn decode_pose_round_trip() {
        use rmpv::Value;
        let msg = Value::Map(vec![
            (Value::from("type"), Value::from("pose")),
            (Value::from("ts_us"), Value::from(123u64)),
            (
                Value::from("position"),
                Value::Array(vec![Value::from(1.0), Value::from(2.0), Value::from(3.0)]),
            ),
            (
                Value::from("orientation_quat"),
                Value::Array(vec![
                    Value::from(1.0),
                    Value::from(0.0),
                    Value::from(0.0),
                    Value::from(0.0),
                ]),
            ),
            (
                Value::from("velocity"),
                Value::Array(vec![Value::from(0.5), Value::from(0.0), Value::from(0.0)]),
            ),
            (Value::from("feature_count"), Value::from(42u64)),
            (Value::from("reset_counter"), Value::from(1u64)),
            (Value::from("state"), Value::from("converged")),
        ]);
        let pose = decode_pose(&msg).unwrap();
        assert_eq!(pose.ts_us, 123);
        assert_eq!(pose.position, (1.0, 2.0, 3.0));
        assert_eq!(pose.feature_count, 42);
        assert_eq!(pose.state, "converged");
    }

    #[test]
    fn decode_pose_rejects_bad_arity() {
        use rmpv::Value;
        let msg = Value::Map(vec![
            (Value::from("type"), Value::from("pose")),
            (Value::from("ts_us"), Value::from(1u64)),
            (
                Value::from("position"),
                Value::Array(vec![Value::from(1.0)]), // wrong arity
            ),
            (Value::from("state"), Value::from("converged")),
        ]);
        assert!(decode_pose(&msg).is_none());
    }

    #[test]
    fn decode_pose_rejects_unknown_state() {
        use rmpv::Value;
        let msg = Value::Map(vec![
            (Value::from("type"), Value::from("pose")),
            (Value::from("state"), Value::from("exploded")),
        ]);
        assert!(decode_pose(&msg).is_none());
    }

    #[test]
    fn frame_ring_writes_header_in_cpp_field_order() {
        // /dev/shm is Linux-only; back the test ring with a tempfile so
        // the SHM-layout assertions run on any host.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("ados-test-ring");
        let mut ring = FrameRing::create_at("ados-test-ring", &path).unwrap();
        let pixels = vec![7u8; 64];
        let slot = ring
            .write(1_000_000, 8, 8, 8, FRAME_FORMAT_GRAY8, &pixels)
            .unwrap();
        assert_eq!(slot.slot_index, 0);
        assert_eq!(slot.seq, 0);
        // Read back the header fields from the mapped region.
        let base = 0usize;
        let seq = u64::from_le_bytes(ring.map[base..base + 8].try_into().unwrap());
        let width = u32::from_le_bytes(ring.map[base + 8..base + 12].try_into().unwrap());
        let payload_size =
            u32::from_le_bytes(ring.map[base + 32..base + 36].try_into().unwrap());
        assert_eq!(seq, 0);
        assert_eq!(width, 8);
        assert_eq!(payload_size, 64);
        // The payload lands right after the header.
        assert_eq!(ring.map[FRAME_SLOT_HEADER_LEN], 7);
    }
}
