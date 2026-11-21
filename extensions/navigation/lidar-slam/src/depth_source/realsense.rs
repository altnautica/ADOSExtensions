//! An Intel RealSense D435i / D455 depth stream through librealsense2.
//!
//! `librealsense2.so` (Apache-2.0, built by `realsense/build-librealsense2.sh`)
//! is opened with `libloading`; nothing links against it at build time, so a
//! host without the library, or without a camera, fails to open with
//! [`DepthSourceError::Unavailable`] instead of at process start.
//!
//! C API used (librealsense v2.58 `include/librealsense2/h/*.h`; every call
//! takes a trailing `rs2_error**`):
//!
//! ```c
//! int rs2_get_api_version(rs2_error**);
//! rs2_context* rs2_create_context(int api_version, rs2_error**);
//! void rs2_delete_context(rs2_context*);
//! rs2_device_list* rs2_query_devices(const rs2_context*, rs2_error**);
//! int rs2_get_device_count(const rs2_device_list*, rs2_error**);
//! void rs2_delete_device_list(rs2_device_list*);
//! rs2_pipeline* rs2_create_pipeline(rs2_context*, rs2_error**);
//! void rs2_delete_pipeline(rs2_pipeline*);
//! rs2_config* rs2_create_config(rs2_error**);
//! void rs2_delete_config(rs2_config*);
//! void rs2_config_enable_stream(rs2_config*, rs2_stream, int index, int width, int height,
//!                               rs2_format, int framerate, rs2_error**);
//! void rs2_config_enable_device(rs2_config*, const char* serial, rs2_error**);
//! rs2_pipeline_profile* rs2_pipeline_start_with_config(rs2_pipeline*, rs2_config*, rs2_error**);
//! void rs2_pipeline_stop(rs2_pipeline*, rs2_error**);
//! void rs2_delete_pipeline_profile(rs2_pipeline_profile*);
//! int rs2_pipeline_try_wait_for_frames(rs2_pipeline*, rs2_frame** out, unsigned int timeout_ms,
//!                                      rs2_error**);
//! int rs2_is_frame_extendable_to(const rs2_frame*, rs2_extension, rs2_error**);
//! int rs2_embedded_frames_count(rs2_frame* composite, rs2_error**);
//! rs2_frame* rs2_extract_frame(rs2_frame* composite, int index, rs2_error**);
//! void rs2_release_frame(rs2_frame*);
//! const rs2_stream_profile* rs2_get_frame_stream_profile(const rs2_frame*, rs2_error**);
//! void rs2_get_stream_profile_data(const rs2_stream_profile*, rs2_stream*, rs2_format*,
//!                                  int* index, int* unique_id, int* framerate, rs2_error**);
//! void rs2_get_video_stream_intrinsics(const rs2_stream_profile*, rs2_intrinsics*, rs2_error**);
//! const void* rs2_get_frame_data(const rs2_frame*, rs2_error**);
//! int rs2_get_frame_width(const rs2_frame*, rs2_error**);
//! int rs2_get_frame_height(const rs2_frame*, rs2_error**);
//! int rs2_get_frame_stride_in_bytes(const rs2_frame*, rs2_error**);
//! float rs2_depth_frame_get_units(const rs2_frame*, rs2_error**);
//! rs2_time_t rs2_get_frame_timestamp(const rs2_frame*, rs2_error**);  // double, ms
//! unsigned long long rs2_get_frame_number(const rs2_frame*, rs2_error**);
//! const char* rs2_get_error_message(const rs2_error*);
//! void rs2_free_error(rs2_error*);
//! ```
//!
//! The context is created with the runtime's own API version, so any
//! librealsense 2.x library loads.

use std::ffi::{c_char, c_int, c_uint, c_void, CStr, CString};
use std::path::PathBuf;
use std::time::Duration;

use libloading::Library;

use super::{DepthFrame, DepthHealth, DepthSource, DepthSourceError, DepthSourceKind, Extrinsic};

/// `RS2_STREAM_DEPTH`.
const STREAM_DEPTH: c_int = 1;
/// `RS2_FORMAT_Z16`.
const FORMAT_Z16: c_int = 1;
/// `RS2_EXTENSION_COMPOSITE_FRAME`.
const EXTENSION_COMPOSITE_FRAME: c_int = 10;

type P = *mut c_void;
type E = *mut P;

/// `rs2_intrinsics`.
#[repr(C)]
#[derive(Default)]
struct Intrinsics {
    width: c_int,
    height: c_int,
    ppx: f32,
    ppy: f32,
    fx: f32,
    fy: f32,
    model: c_int,
    coeffs: [f32; 5],
}

fn unavailable(msg: impl Into<String>) -> DepthSourceError {
    DepthSourceError::Unavailable(msg.into())
}

fn sym<T: Copy>(lib: &Library, name: &str) -> Result<T, DepthSourceError> {
    let z = CString::new(name).map_err(|_| unavailable(name.to_string()))?;
    // SAFETY: each caller names a symbol whose C signature matches `T`
    // (the table in the module docs).
    unsafe { lib.get::<T>(z.as_bytes_with_nul()) }
        .map(|s| *s)
        .map_err(|e| unavailable(format!("librealsense2 lacks {name}: {e}")))
}

/// The resolved C API: each field is the C function `rs2_<field>`.
macro_rules! api {
    ($($field:ident: $ty:ty,)*) => {
        struct Api {
            $($field: $ty,)*
        }

        /// Every `rs2_*` symbol the binding resolves.
        pub const SYMBOLS: &[&str] = &[$(concat!("rs2_", stringify!($field))),*];

        impl Api {
            fn load(lib: &Library) -> Result<Self, DepthSourceError> {
                Ok(Self {
                    $($field: sym(lib, concat!("rs2_", stringify!($field)))?,)*
                })
            }
        }
    };
}

api! {
    get_api_version: unsafe extern "C" fn(E) -> c_int,
    create_context: unsafe extern "C" fn(c_int, E) -> P,
    delete_context: unsafe extern "C" fn(P),
    query_devices: unsafe extern "C" fn(P, E) -> P,
    get_device_count: unsafe extern "C" fn(P, E) -> c_int,
    delete_device_list: unsafe extern "C" fn(P),
    create_pipeline: unsafe extern "C" fn(P, E) -> P,
    delete_pipeline: unsafe extern "C" fn(P),
    create_config: unsafe extern "C" fn(E) -> P,
    delete_config: unsafe extern "C" fn(P),
    config_enable_stream: unsafe extern "C" fn(P, c_int, c_int, c_int, c_int, c_int, c_int, E),
    config_enable_device: unsafe extern "C" fn(P, *const c_char, E),
    pipeline_start_with_config: unsafe extern "C" fn(P, P, E) -> P,
    pipeline_stop: unsafe extern "C" fn(P, E),
    delete_pipeline_profile: unsafe extern "C" fn(P),
    pipeline_try_wait_for_frames: unsafe extern "C" fn(P, *mut P, c_uint, E) -> c_int,
    is_frame_extendable_to: unsafe extern "C" fn(P, c_int, E) -> c_int,
    embedded_frames_count: unsafe extern "C" fn(P, E) -> c_int,
    extract_frame: unsafe extern "C" fn(P, c_int, E) -> P,
    release_frame: unsafe extern "C" fn(P),
    get_frame_stream_profile: unsafe extern "C" fn(P, E) -> P,
    get_stream_profile_data:
        unsafe extern "C" fn(P, *mut c_int, *mut c_int, *mut c_int, *mut c_int, *mut c_int, E),
    get_video_stream_intrinsics: unsafe extern "C" fn(P, *mut Intrinsics, E),
    get_frame_data: unsafe extern "C" fn(P, E) -> *const c_void,
    get_frame_width: unsafe extern "C" fn(P, E) -> c_int,
    get_frame_height: unsafe extern "C" fn(P, E) -> c_int,
    get_frame_stride_in_bytes: unsafe extern "C" fn(P, E) -> c_int,
    depth_frame_get_units: unsafe extern "C" fn(P, E) -> f32,
    get_frame_timestamp: unsafe extern "C" fn(P, E) -> f64,
    // `unsigned long long`: 64 bits on every supported target.
    get_frame_number: unsafe extern "C" fn(P, E) -> u64,
    get_error_message: unsafe extern "C" fn(P) -> *const c_char,
    free_error: unsafe extern "C" fn(P),
}

impl Api {
    /// Run `f` with a fresh error slot; the library's message on failure.
    fn call<R>(&self, f: impl FnOnce(E) -> R) -> Result<R, String> {
        let mut err: P = std::ptr::null_mut();
        let r = f(&mut err);
        if err.is_null() {
            return Ok(r);
        }
        // SAFETY: `err` is an `rs2_error*` the library just returned; its
        // message lives until `rs2_free_error`.
        let msg = unsafe {
            let m = (self.get_error_message)(err);
            let s = if m.is_null() {
                "unknown librealsense2 error".to_string()
            } else {
                CStr::from_ptr(m).to_string_lossy().into_owned()
            };
            (self.free_error)(err);
            s
        };
        Err(msg)
    }
}

#[derive(Debug, Clone)]
pub struct RealsenseConfig {
    pub library: PathBuf,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    /// Empty: the first camera found.
    pub serial: String,
    pub extrinsic: Extrinsic,
}

/// A streaming RealSense depth camera.
pub struct RealsenseSource {
    api: Api,
    ctx: P,
    pipe: P,
    config: P,
    profile: P,
    extrinsic: Extrinsic,
    last_number: Option<u64>,
    spare: Vec<u16>,
    health: DepthHealth,
    // Dropped last: the function pointers above live in it.
    _lib: Library,
}

// SAFETY: the handles are owned by this value and used from one thread at a
// time (`&mut self`); librealsense objects are not tied to their creating
// thread.
unsafe impl Send for RealsenseSource {}

impl RealsenseSource {
    /// Load the library, find a camera and start its depth stream.
    pub fn open(cfg: &RealsenseConfig) -> Result<Self, DepthSourceError> {
        // SAFETY: loading runs the library's initialisers; librealsense2's
        // only set up its own globals.
        let lib = unsafe { Library::new(&cfg.library) }.map_err(|e| {
            unavailable(format!(
                "cannot load librealsense2 {}: {e}",
                cfg.library.display()
            ))
        })?;
        let api = Api::load(&lib)?;
        let mut me = Self {
            api,
            ctx: std::ptr::null_mut(),
            pipe: std::ptr::null_mut(),
            config: std::ptr::null_mut(),
            profile: std::ptr::null_mut(),
            extrinsic: cfg.extrinsic,
            last_number: None,
            spare: Vec::new(),
            health: DepthHealth::default(),
            _lib: lib,
        };
        me.start(cfg).map_err(unavailable)?;
        Ok(me)
    }

    fn start(&mut self, cfg: &RealsenseConfig) -> Result<(), String> {
        let a = &self.api;
        // SAFETY (every call below): C API functions from the loaded library
        // with the documented signatures, given live handles or null-checked
        // results of earlier calls.
        let version = a.call(|e| unsafe { (a.get_api_version)(e) })?;
        self.ctx = a.call(|e| unsafe { (a.create_context)(version, e) })?;
        let list = a.call(|e| unsafe { (a.query_devices)(self.ctx, e) })?;
        let count = a.call(|e| unsafe { (a.get_device_count)(list, e) });
        unsafe { (a.delete_device_list)(list) };
        if count? < 1 {
            return Err("no RealSense camera connected".into());
        }
        self.pipe = a.call(|e| unsafe { (a.create_pipeline)(self.ctx, e) })?;
        self.config = a.call(|e| unsafe { (a.create_config)(e) })?;
        if !cfg.serial.is_empty() {
            let serial = CString::new(cfg.serial.as_str()).map_err(|_| "serial has a NUL")?;
            a.call(|e| unsafe { (a.config_enable_device)(self.config, serial.as_ptr(), e) })?;
        }
        let (w, h, fps) = (cfg.width as c_int, cfg.height as c_int, cfg.fps as c_int);
        a.call(|e| unsafe {
            (a.config_enable_stream)(self.config, STREAM_DEPTH, 0, w, h, FORMAT_Z16, fps, e)
        })?;
        self.profile = a
            .call(|e| unsafe { (a.pipeline_start_with_config)(self.pipe, self.config, e) })
            .map_err(|m| format!("depth {w}x{h} at {fps} fps refused: {m}"))?;
        Ok(())
    }

    /// The depth frame in `frame` (itself, or the one inside a frameset).
    /// Takes over `frame`'s reference; the caller releases the one returned.
    fn depth_of(&self, frame: P) -> Result<Option<P>, String> {
        let a = &self.api;
        let release = |f: P| unsafe { (a.release_frame)(f) };
        let composite =
            a.call(|e| unsafe { (a.is_frame_extendable_to)(frame, EXTENSION_COMPOSITE_FRAME, e) });
        if composite != Ok(1) {
            let depth = composite.and_then(|_| self.is_depth(frame));
            if depth != Ok(true) {
                release(frame);
            }
            return depth.map(|d| d.then_some(frame));
        }
        let mut found = None;
        let mut scan = || -> Result<(), String> {
            let n = a.call(|e| unsafe { (a.embedded_frames_count)(frame, e) })?;
            for i in 0..n {
                let f = a.call(|e| unsafe { (a.extract_frame)(frame, i, e) })?;
                match self.is_depth(f) {
                    Ok(true) if found.is_none() => found = Some(f),
                    Ok(_) => release(f),
                    Err(m) => {
                        release(f);
                        return Err(m);
                    }
                }
            }
            Ok(())
        };
        let scanned = scan();
        release(frame);
        match scanned {
            Ok(()) => Ok(found),
            Err(m) => {
                if let Some(f) = found {
                    release(f);
                }
                Err(m)
            }
        }
    }

    fn is_depth(&self, f: P) -> Result<bool, String> {
        let a = &self.api;
        let prof = a.call(|e| unsafe { (a.get_frame_stream_profile)(f, e) })?;
        let (mut stream, mut format, mut index, mut uid, mut rate) = (0, 0, 0, 0, 0);
        a.call(|e| unsafe {
            (a.get_stream_profile_data)(
                prof,
                &mut stream,
                &mut format,
                &mut index,
                &mut uid,
                &mut rate,
                e,
            )
        })?;
        Ok(stream == STREAM_DEPTH && format == FORMAT_Z16)
    }

    /// Copy depth frame `f` out; `f` stays owned by the caller.
    fn copy(&mut self, f: P) -> Result<DepthFrame, String> {
        let a = &self.api;
        let w = a.call(|e| unsafe { (a.get_frame_width)(f, e) })?;
        let h = a.call(|e| unsafe { (a.get_frame_height)(f, e) })?;
        let stride = a.call(|e| unsafe { (a.get_frame_stride_in_bytes)(f, e) })?;
        let units = a.call(|e| unsafe { (a.depth_frame_get_units)(f, e) })?;
        let ts_ms = a.call(|e| unsafe { (a.get_frame_timestamp)(f, e) })?;
        let number = a.call(|e| unsafe { (a.get_frame_number)(f, e) })?;
        let prof = a.call(|e| unsafe { (a.get_frame_stream_profile)(f, e) })?;
        let mut k = Intrinsics::default();
        a.call(|e| unsafe { (a.get_video_stream_intrinsics)(prof, &mut k, e) })?;
        let data = a.call(|e| unsafe { (a.get_frame_data)(f, e) })?;
        let (wu, hu) = (w.max(0) as usize, h.max(0) as usize);
        let stride = usize::try_from(stride).unwrap_or(0);
        if data.is_null() || wu == 0 || hu == 0 || stride < wu * 2 {
            return Err(format!("bad depth frame {w}x{h} stride {stride}"));
        }
        // SAFETY: a Z16 frame's data is `h` rows of `stride` bytes, alive
        // while the caller holds `f`.
        let bytes = unsafe { std::slice::from_raw_parts(data as *const u8, stride * hu) };
        let mut depth_mm = std::mem::take(&mut self.spare);
        depth_mm.clear();
        depth_mm.reserve(wu * hu);
        // Device units to millimetres (1 on a D4xx at its default 1 mm unit).
        let scale = units * 1000.0;
        let unit_mm = (scale - 1.0).abs() < 1e-4;
        for row in bytes.chunks_exact(stride) {
            depth_mm.extend(row[..wu * 2].as_chunks::<2>().0.iter().map(|&p| {
                let raw = u16::from_le_bytes(p);
                if unit_mm {
                    raw
                } else {
                    (f32::from(raw) * scale).round().min(f32::from(u16::MAX)) as u16
                }
            }));
        }
        if let Some(prev) = self.last_number {
            self.health.drops += number.saturating_sub(prev).saturating_sub(1);
        }
        self.last_number = Some(number);
        let stamp_ns = crate::timebase::now_ns();
        self.health.frames += 1;
        self.health.last_stamp_ns = stamp_ns;
        Ok(DepthFrame {
            stamp_ns,
            source_ns: (ts_ms.max(0.0) * 1e6) as u64,
            seq: number,
            w: wu as u32,
            h: hu as u32,
            fx: k.fx,
            fy: k.fy,
            cx: k.ppx,
            cy: k.ppy,
            depth_mm,
            extrinsic: self.extrinsic,
        })
    }
}

impl DepthSource for RealsenseSource {
    fn kind(&self) -> DepthSourceKind {
        DepthSourceKind::Realsense
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>, DepthSourceError> {
        let a = &self.api;
        let ms = timeout.as_millis().min(u128::from(u32::MAX)) as c_uint;
        let mut frame: P = std::ptr::null_mut();
        let got = a
            .call(|e| unsafe { (a.pipeline_try_wait_for_frames)(self.pipe, &mut frame, ms, e) })
            .map_err(DepthSourceError::Stream)?;
        if got == 0 || frame.is_null() {
            return Ok(None);
        }
        let Some(depth) = self.depth_of(frame).map_err(DepthSourceError::Stream)? else {
            self.health.rejected += 1;
            return Ok(None);
        };
        let out = self.copy(depth);
        unsafe { (self.api.release_frame)(depth) };
        match out {
            Ok(f) => Ok(Some(f)),
            Err(m) => {
                self.health.rejected += 1;
                Err(DepthSourceError::Stream(m))
            }
        }
    }

    fn health(&self) -> DepthHealth {
        self.health
    }

    fn recycle(&mut self, frame: DepthFrame) {
        if frame.depth_mm.capacity() > self.spare.capacity() {
            self.spare = frame.depth_mm;
        }
    }
}

impl Drop for RealsenseSource {
    fn drop(&mut self) {
        let a = &self.api;
        // SAFETY: each handle was created by this value and is freed once.
        unsafe {
            if !self.profile.is_null() {
                let _ = a.call(|e| (a.pipeline_stop)(self.pipe, e));
                (a.delete_pipeline_profile)(self.profile);
            }
            if !self.config.is_null() {
                (a.delete_config)(self.config);
            }
            if !self.pipe.is_null() {
                (a.delete_pipeline)(self.pipe);
            }
            if !self.ctx.is_null() {
                (a.delete_context)(self.ctx);
            }
        }
    }
}
