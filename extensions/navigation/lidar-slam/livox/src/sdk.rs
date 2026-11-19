//! Run-time binding to the Livox SDK2 shared library.
//!
//! `liblivox_lidar_sdk_shared.so` (Livox-SDK2, MIT, built by
//! `build-sdk2.sh`) is opened with `libloading`; nothing links against it at
//! build time, so a host without the library fails here with
//! [`SdkError::Load`] instead of at process start.
//!
//! C API used (Livox-SDK2 v1.2.5 `include/livox_lidar_api.h`, `extern "C"`;
//! callback types from `include/livox_lidar_def.h`):
//!
//! ```c
//! bool LivoxLidarSdkInit(const char* path, const char* host_ip,
//!                        const LivoxLidarLoggerCfgInfo* log_cfg_info);
//! bool LivoxLidarSdkStart();
//! void LivoxLidarSdkUninit();
//! void DisableLivoxSdkConsoleLogger();
//! void SetLivoxLidarPointCloudCallBack(LivoxLidarPointCloudCallBack cb, void* client_data);
//! void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void* client_data);
//! void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data);
//! livox_status SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode,
//!                                    LivoxLidarAsyncControlCallback cb, void* client_data);
//! livox_status EnableLivoxLidarImuData(uint32_t handle,
//!                                      LivoxLidarAsyncControlCallback cb, void* client_data);
//! typedef void (*LivoxLidarPointCloudCallBack)(const uint32_t handle, const uint8_t dev_type,
//!                                              LivoxLidarEthernetPacket* data, void* client_data);
//! typedef void (*LivoxLidarInfoChangeCallback)(const uint32_t handle,
//!                                              const LivoxLidarInfo* info, void* client_data);
//! typedef void (*LivoxLidarAsyncControlCallback)(livox_status status, uint32_t handle,
//!                                                LivoxLidarAsyncControlResponse* response,
//!                                                void* client_data);
//! ```
//!
//! The SDK keeps global state, so one [`SdkDriver`] may exist per process.
//! Callbacks run on SDK threads: they only stamp the host monotonic time, copy
//! the packet (its `length` field bounds the copy) into a pooled buffer and
//! queue it. A decode thread runs one [`Decoder`] per device handle and hands
//! every [`Event`] to the caller's sink.

use std::collections::HashMap;
use std::ffi::{c_char, c_void, CString};
use std::os::unix::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use libloading::Library;

use crate::decoder::{Decoder, DecoderConfig, DecoderStats, Event};
use crate::packet::{declared_len, HEADER_LEN};

/// `LivoxLidarWorkMode::kLivoxLidarNormal`.
const WORK_MODE_NORMAL: i32 = 0x01;
/// Largest packet copied out of a callback (a UDP datagram's limit).
const MAX_PACKET: usize = 65_507;
/// Buffers kept for reuse.
const POOL_CAP: usize = 256;

type PacketCb = unsafe extern "C" fn(u32, u8, *mut u8, *mut c_void);
type InfoChangeCb = unsafe extern "C" fn(u32, *const c_void, *mut c_void);
type AsyncCtlCb = unsafe extern "C" fn(i32, u32, *mut c_void, *mut c_void);
type InitFn = unsafe extern "C" fn(*const c_char, *const c_char, *const c_void) -> bool;
type StartFn = unsafe extern "C" fn() -> bool;
type VoidFn = unsafe extern "C" fn();
type SetPacketCbFn = unsafe extern "C" fn(Option<PacketCb>, *mut c_void);
type SetInfoChangeCbFn = unsafe extern "C" fn(Option<InfoChangeCb>, *mut c_void);
type SetWorkModeFn = unsafe extern "C" fn(u32, i32, Option<AsyncCtlCb>, *mut c_void) -> i32;
type EnableImuFn = unsafe extern "C" fn(u32, Option<AsyncCtlCb>, *mut c_void) -> i32;

static ACTIVE: AtomicBool = AtomicBool::new(false);

#[derive(Debug, thiserror::Error)]
pub enum SdkError {
    #[error("a Livox SDK2 driver is already running in this process")]
    AlreadyRunning,
    #[error("cannot load Livox SDK2 library {path}: {source}")]
    Load {
        path: PathBuf,
        source: libloading::Error,
    },
    #[error("Livox SDK2 library lacks symbol {name}: {source}")]
    Symbol {
        name: &'static str,
        source: libloading::Error,
    },
    #[error("path contains a NUL byte: {0}")]
    Path(PathBuf),
    #[error("LivoxLidarSdkInit failed for config {0}")]
    Init(PathBuf),
    #[error("LivoxLidarSdkStart failed")]
    Start,
    #[error("cannot start the decode thread: {0}")]
    Thread(std::io::Error),
}

#[derive(Debug, Clone)]
pub struct SdkDriverConfig {
    /// `liblivox_lidar_sdk_shared.so`.
    pub library: PathBuf,
    /// JSON written from [`crate::SdkNetConfig`].
    pub config_json: PathBuf,
    pub decoder: DecoderConfig,
    /// Ask each discovered LiDAR to push IMU data.
    pub enable_imu: bool,
    /// Packets queued between the callbacks and the decode thread.
    pub queue: usize,
}

/// An [`Event`] from one device (`handle` is the SDK's device handle).
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceEvent {
    pub handle: u32,
    pub event: Event,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SdkStats {
    /// Packets dropped because the decode queue was full or oversized.
    pub dropped: u64,
    pub devices: Vec<(u32, DecoderStats)>,
}

struct RawPacket {
    handle: u32,
    host_rx_ns: u64,
    bytes: Vec<u8>,
}

struct Ctx {
    tx: SyncSender<RawPacket>,
    pool: Arc<Mutex<Vec<Vec<u8>>>>,
    dropped: AtomicU64,
    set_work_mode: SetWorkModeFn,
    enable_imu: Option<EnableImuFn>,
}

pub struct SdkDriver {
    uninit: VoidFn,
    ctx: *mut Ctx,
    thread: Option<JoinHandle<()>>,
    stats: Arc<Mutex<Vec<(u32, DecoderStats)>>>,
    // Dropped last: the function pointers above live in it.
    _lib: Library,
}

// SAFETY: `ctx` is only dereferenced by SDK callbacks (through `client_data`)
// and freed in `Drop` after `LivoxLidarSdkUninit` has stopped them.
unsafe impl Send for SdkDriver {}

fn mono_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid out-pointer; CLOCK_MONOTONIC always exists.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

fn sym<T: Copy>(lib: &Library, name: &'static str) -> Result<T, SdkError> {
    let mut z = Vec::with_capacity(name.len() + 1);
    z.extend_from_slice(name.as_bytes());
    z.push(0);
    // SAFETY: the caller names a symbol whose C type matches `T`.
    unsafe { lib.get::<T>(&z) }
        .map(|s| *s)
        .map_err(|source| SdkError::Symbol { name, source })
}

fn c_path(p: &Path) -> Result<CString, SdkError> {
    CString::new(p.as_os_str().as_bytes()).map_err(|_| SdkError::Path(p.to_path_buf()))
}

unsafe extern "C" fn on_packet(handle: u32, _dev_type: u8, data: *mut u8, client: *mut c_void) {
    let _ = std::panic::catch_unwind(|| {
        if data.is_null() || client.is_null() {
            return;
        }
        let host_rx_ns = mono_ns();
        // SAFETY: `client` is the `Ctx` registered with the callback; it
        // outlives every callback (see `SdkDriver::drop`).
        let ctx = unsafe { &*(client as *const Ctx) };
        // SAFETY: the SDK hands a packet of at least its fixed header.
        let head = unsafe { std::slice::from_raw_parts(data, 3) };
        let len = declared_len(head).unwrap_or(0);
        if !(HEADER_LEN..=MAX_PACKET).contains(&len) {
            ctx.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        // SAFETY: `length` is the size of the UDP data segment the SDK holds.
        let src = unsafe { std::slice::from_raw_parts(data, len) };
        let mut bytes = ctx
            .pool
            .lock()
            .ok()
            .and_then(|mut p| p.pop())
            .unwrap_or_default();
        bytes.clear();
        bytes.extend_from_slice(src);
        let pkt = RawPacket {
            handle,
            host_rx_ns,
            bytes,
        };
        if let Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) = ctx.tx.try_send(pkt) {
            ctx.dropped.fetch_add(1, Ordering::Relaxed);
        }
    });
}

unsafe extern "C" fn on_async(status: i32, handle: u32, _resp: *mut c_void, _client: *mut c_void) {
    let _ = std::panic::catch_unwind(|| {
        if status != 0 {
            tracing::warn!(handle, status, "livox control request failed");
        }
    });
}

unsafe extern "C" fn on_info_change(handle: u32, _info: *const c_void, client: *mut c_void) {
    let _ = std::panic::catch_unwind(|| {
        if client.is_null() {
            return;
        }
        // SAFETY: as in `on_packet`.
        let ctx = unsafe { &*(client as *const Ctx) };
        tracing::info!(handle, "livox device found; requesting normal work mode");
        // SAFETY: SDK functions resolved from the loaded library; the sample
        // driver issues the same calls from this callback.
        unsafe {
            (ctx.set_work_mode)(
                handle,
                WORK_MODE_NORMAL,
                Some(on_async),
                std::ptr::null_mut(),
            );
            if let Some(enable) = ctx.enable_imu {
                enable(handle, Some(on_async), std::ptr::null_mut());
            }
        }
    });
}

fn decode_loop(
    rx: Receiver<RawPacket>,
    pool: Arc<Mutex<Vec<Vec<u8>>>>,
    stats: Arc<Mutex<Vec<(u32, DecoderStats)>>>,
    cfg: DecoderConfig,
    mut sink: impl FnMut(DeviceEvent),
) {
    let mut decoders: HashMap<u32, Decoder> = HashMap::new();
    let mut events = Vec::new();
    let publish = |decoders: &HashMap<u32, Decoder>| {
        if let Ok(mut s) = stats.lock() {
            s.clear();
            s.extend(decoders.iter().map(|(h, d)| (*h, d.stats())));
        }
    };
    let mut since_publish = 0u32;
    while let Ok(pkt) = rx.recv() {
        let dec = decoders
            .entry(pkt.handle)
            .or_insert_with(|| Decoder::new(cfg));
        if let Err(e) = dec.push(&pkt.bytes, pkt.host_rx_ns, &mut events) {
            tracing::debug!(handle = pkt.handle, error = %e, "livox packet rejected");
        }
        if let Ok(mut p) = pool.lock() {
            if p.len() < POOL_CAP {
                p.push(pkt.bytes);
            }
        }
        for event in events.drain(..) {
            sink(DeviceEvent {
                handle: pkt.handle,
                event,
            });
        }
        since_publish += 1;
        if since_publish >= 100 {
            since_publish = 0;
            publish(&decoders);
        }
    }
    for (handle, dec) in decoders.iter_mut() {
        dec.flush(&mut events);
        for event in events.drain(..) {
            sink(DeviceEvent {
                handle: *handle,
                event,
            });
        }
    }
    publish(&decoders);
}

impl SdkDriver {
    /// Load the library, start the SDK and deliver every decoded event to
    /// `sink` on the decode thread.
    pub fn start<F>(cfg: SdkDriverConfig, sink: F) -> Result<Self, SdkError>
    where
        F: FnMut(DeviceEvent) + Send + 'static,
    {
        if ACTIVE
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(SdkError::AlreadyRunning);
        }
        Self::start_inner(cfg, sink).inspect_err(|_| ACTIVE.store(false, Ordering::Release))
    }

    fn start_inner<F>(cfg: SdkDriverConfig, sink: F) -> Result<Self, SdkError>
    where
        F: FnMut(DeviceEvent) + Send + 'static,
    {
        let config_c = c_path(&cfg.config_json)?;
        // SAFETY: loading runs the library's initialisers; the SDK's are
        // plain C++ static constructors.
        let lib = unsafe { Library::new(&cfg.library) }.map_err(|source| SdkError::Load {
            path: cfg.library.clone(),
            source,
        })?;
        let init: InitFn = sym(&lib, "LivoxLidarSdkInit")?;
        let start: StartFn = sym(&lib, "LivoxLidarSdkStart")?;
        let uninit: VoidFn = sym(&lib, "LivoxLidarSdkUninit")?;
        let set_points: SetPacketCbFn = sym(&lib, "SetLivoxLidarPointCloudCallBack")?;
        let set_imu: SetPacketCbFn = sym(&lib, "SetLivoxLidarImuDataCallback")?;
        let set_info: SetInfoChangeCbFn = sym(&lib, "SetLivoxLidarInfoChangeCallback")?;
        let set_work_mode: SetWorkModeFn = sym(&lib, "SetLivoxLidarWorkMode")?;
        let enable_imu: EnableImuFn = sym(&lib, "EnableLivoxLidarImuData")?;
        let quiet: Option<VoidFn> = sym(&lib, "DisableLivoxSdkConsoleLogger").ok();

        let (tx, rx) = sync_channel(cfg.queue.max(1));
        let pool = Arc::new(Mutex::new(Vec::new()));
        let stats = Arc::new(Mutex::new(Vec::new()));
        let ctx = Box::into_raw(Box::new(Ctx {
            tx,
            pool: pool.clone(),
            dropped: AtomicU64::new(0),
            set_work_mode,
            enable_imu: cfg.enable_imu.then_some(enable_imu),
        }));
        let thread = {
            let (stats, dcfg) = (stats.clone(), cfg.decoder);
            std::thread::Builder::new()
                .name("livox-decode".into())
                .spawn(move || decode_loop(rx, pool, stats, dcfg, sink))
        };
        let thread = match thread {
            Ok(t) => t,
            Err(e) => {
                // SAFETY: never shared with the SDK yet.
                drop(unsafe { Box::from_raw(ctx) });
                return Err(SdkError::Thread(e));
            }
        };
        let driver = Self {
            uninit,
            ctx,
            thread: Some(thread),
            stats,
            _lib: lib,
        };
        let client = ctx as *mut c_void;
        // SAFETY: symbols resolved above with their documented C types; the
        // strings outlive the calls; `client` stays valid until `Drop`.
        unsafe {
            if let Some(q) = quiet {
                q();
            }
            if !init(config_c.as_ptr(), c"".as_ptr(), std::ptr::null()) {
                // `driver` drops here and calls LivoxLidarSdkUninit, as the
                // SDK's own sample does after a failed init.
                return Err(SdkError::Init(cfg.config_json));
            }
            set_points(Some(on_packet), client);
            set_imu(Some(on_packet), client);
            set_info(Some(on_info_change), client);
            if !start() {
                return Err(SdkError::Start);
            }
        }
        Ok(driver)
    }

    pub fn stats(&self) -> SdkStats {
        // SAFETY: `ctx` is live until `Drop`.
        let dropped = unsafe { &*self.ctx }.dropped.load(Ordering::Relaxed);
        SdkStats {
            dropped,
            devices: self.stats.lock().map(|s| s.clone()).unwrap_or_default(),
        }
    }
}

impl Drop for SdkDriver {
    fn drop(&mut self) {
        // SAFETY: stops every SDK thread, so no callback runs afterwards.
        unsafe { (self.uninit)() };
        // SAFETY: allocated in `start_inner`, no longer reachable by the SDK.
        drop(unsafe { Box::from_raw(self.ctx) });
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        ACTIVE.store(false, Ordering::Release);
    }
}
