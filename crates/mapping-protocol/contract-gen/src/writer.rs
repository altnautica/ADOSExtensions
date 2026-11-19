//! [`SessionWriter`]: records one capture session directory.
//!
//! Messages go to `segments/NNNNNN.mcap` (protobuf encoding, lz4 chunks); a
//! segment is closed and the next one opened when it reaches
//! [`SegmentLimits::max_bytes`] or has recorded [`SegmentLimits::max_duration_ns`]
//! of elapsed time since its first message. Elapsed time is the sum of the
//! forward steps between messages: a message older than the newest one (out
//! of order) and a forward jump longer than the duration limit (a stamp off
//! the session timebase) add nothing.
//! Every segment carries `rig.json` and a snapshot of the manifest taken when
//! the segment opened as attachments; `manifest.json` in the directory,
//! written by [`SessionWriter::finish`], is the authoritative copy.

use std::collections::{BTreeMap, HashMap};
use std::fs::{self, File};
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};

use mcap::records::MessageHeader;

use crate::schema::{Manifest, Rig, Segment};
use crate::{topics, write_checksums, Error, Result, FILE_DESCRIPTOR_SET, MCAP_PROFILE};

/// When a segment is rolled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SegmentLimits {
    /// Roll once the segment file reaches this many bytes.
    pub max_bytes: u64,
    /// Roll once this much time has elapsed since the segment's first
    /// message (see the module notes for what counts as elapsed).
    pub max_duration_ns: u64,
}

impl Default for SegmentLimits {
    /// 4 GiB or 10 minutes, whichever comes first.
    fn default() -> Self {
        Self {
            max_bytes: 4 << 30,
            max_duration_ns: 600_000_000_000,
        }
    }
}

/// Where a segment's bytes go: a buffered file in production
/// ([`SessionWriter::with_sink`] swaps it, to inject a full disk in tests).
pub trait SegmentSink: Write + Seek + Send {
    /// Make everything written so far durable.
    fn sync(&mut self) -> std::io::Result<()>;
}

impl SegmentSink for BufWriter<File> {
    fn sync(&mut self) -> std::io::Result<()> {
        self.flush()?;
        self.get_ref().sync_all()
    }
}

/// Opens the sink of the segment file at the given path.
pub type OpenSink = Box<dyn Fn(&Path) -> std::io::Result<Box<dyn SegmentSink>> + Send>;

fn open_file(path: &Path) -> std::io::Result<Box<dyn SegmentSink>> {
    Ok(Box::new(BufWriter::new(File::create(path)?)))
}

/// The first I/O error of a session's segment sinks, shared by the writer
/// and every segment it opens.
type Fault = Arc<OnceLock<(std::io::ErrorKind, String)>>;

fn latch(fault: &Fault, e: &std::io::Error) {
    let _ = fault.set((e.kind(), e.to_string()));
}

/// `Write + Seek` that records the furthest byte written.
///
/// After the sink's first error (a full disk) every later write, flush and
/// seek succeeds without touching the sink, and the session writer reports
/// the latched error instead. The mcap writer is never shown an error: one
/// that lands part way through starting a chunk leaves it without its
/// stream, and its drop then panics, which aborts a release build (panic =
/// abort) and with it every lane of the process that owns the session.
struct CountingWriter {
    inner: Box<dyn SegmentSink>,
    pos: u64,
    high: Arc<AtomicU64>,
    fault: Fault,
}

impl CountingWriter {
    fn advance(&mut self, n: usize) {
        self.pos += n as u64;
        self.high.fetch_max(self.pos, Ordering::Relaxed);
    }
}

impl Write for CountingWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        if self.fault.get().is_none() {
            match self.inner.write(buf) {
                Ok(n) => {
                    self.advance(n);
                    return Ok(n);
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => return Err(e),
                Err(e) => latch(&self.fault, &e),
            }
        }
        self.advance(buf.len());
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        if self.fault.get().is_none() {
            if let Err(e) = self.inner.flush() {
                latch(&self.fault, &e);
            }
        }
        Ok(())
    }
}

impl Seek for CountingWriter {
    fn seek(&mut self, to: SeekFrom) -> std::io::Result<u64> {
        if self.fault.get().is_none() {
            match self.inner.seek(to) {
                Ok(p) => {
                    self.pos = p;
                    return Ok(p);
                }
                Err(e) => latch(&self.fault, &e),
            }
        }
        let from = |base: u64, d: i64| base.saturating_add_signed(d);
        self.pos = match to {
            SeekFrom::Start(p) => p,
            SeekFrom::Current(d) => from(self.pos, d),
            SeekFrom::End(d) => from(self.high.load(Ordering::Relaxed), d),
        };
        Ok(self.pos)
    }
}

struct OpenSegment {
    writer: mcap::Writer<CountingWriter>,
    high: Arc<AtomicU64>,
    file: String,
    t_start_ns: u64,
    t_end_ns: u64,
    /// The stamp elapsed time is counted from (the newest counted message).
    last_ns: u64,
    /// Elapsed time since the segment's first message.
    elapsed_ns: u64,
    channels: HashMap<String, u16>,
    seq: u32,
}

impl OpenSegment {
    /// Elapsed time a message at `t_ns` adds, and the stamp counted from
    /// after it.
    fn step(&self, t_ns: u64, window: u64) -> (u64, u64) {
        match t_ns.checked_sub(self.last_ns) {
            Some(0) | None => (0, self.last_ns),
            Some(d) if d <= window => (d, t_ns),
            // A jump past the window is a stamp off the timebase: ignore it.
            Some(_) => (0, self.last_ns),
        }
    }
}

/// Writes one session directory. Call [`SessionWriter::finish`] to close the
/// last segment and write `manifest.json` and `checksums.b3`.
pub struct SessionWriter {
    dir: PathBuf,
    manifest: Manifest,
    rig_json: Vec<u8>,
    limits: SegmentLimits,
    segment: Option<OpenSegment>,
    next_index: u32,
    buf: Vec<u8>,
    open_sink: OpenSink,
    fault: Fault,
}

impl SessionWriter {
    /// Create `<root>/<manifest.session_id>/` and write `rig.json`. Fails if the
    /// directory already holds a session or the manifest names another rig.
    pub fn create(root: &Path, mut manifest: Manifest, rig: &Rig) -> Result<Self> {
        let id = &manifest.session_id;
        if id.is_empty() || id.contains(['/', '\\']) || id.starts_with('.') {
            return Err(Error::Invalid(format!(
                "bad session id `{}`",
                manifest.session_id
            )));
        }
        if manifest.rig.rig_id != rig.rig_id || manifest.rig.calib_version != rig.calib_version {
            return Err(Error::Invalid(
                "manifest rig does not match rig.json".into(),
            ));
        }
        rig.check()?;
        let dir = root.join(&manifest.session_id);
        if dir.join("manifest.json").exists() || dir.join("rig.json").exists() {
            return Err(Error::Invalid(format!(
                "session {} already exists",
                manifest.session_id
            )));
        }
        fs::create_dir_all(dir.join("segments"))?;
        let rig_json = serde_json::to_vec_pretty(rig)?;
        fs::write(dir.join("rig.json"), &rig_json)?;
        manifest.segments.clear();
        Ok(Self {
            dir,
            manifest,
            rig_json,
            limits: SegmentLimits::default(),
            segment: None,
            next_index: 0,
            buf: Vec::new(),
            open_sink: Box::new(open_file),
            fault: Fault::default(),
        })
    }

    /// Replace the segment roll limits.
    pub fn with_limits(mut self, limits: SegmentLimits) -> Self {
        self.limits = limits;
        self
    }

    /// Send segment bytes to the sinks `open` returns instead of files.
    pub fn with_sink(mut self, open: OpenSink) -> Self {
        self.open_sink = open;
        self
    }

    /// The session directory.
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// The manifest as it will be written, for operator rooms and health
    /// updates during capture. `segments` is owned by the writer.
    pub fn manifest_mut(&mut self) -> &mut Manifest {
        &mut self.manifest
    }

    /// Record a protobuf message on `topic` at `t_ns` (session timebase).
    pub fn write_proto<M: prost::Message + prost::Name>(
        &mut self,
        topic: &str,
        t_ns: u64,
        msg: &M,
    ) -> Result<()> {
        let mut buf = std::mem::take(&mut self.buf);
        buf.clear();
        msg.encode(&mut buf).expect("Vec<u8> grows");
        let r = self.write_encoded(topic, Some(&M::full_name()), "protobuf", t_ns, &buf);
        self.buf = buf;
        r
    }

    /// Record a schemaless JSON message (the `/health` channel).
    pub fn write_json(&mut self, topic: &str, t_ns: u64, value: &serde_json::Value) -> Result<()> {
        let data = serde_json::to_vec(value)?;
        self.write_encoded(topic, None, "json", t_ns, &data)
    }

    fn write_encoded(
        &mut self,
        topic: &str,
        schema: Option<&str>,
        encoding: &str,
        t_ns: u64,
        data: &[u8],
    ) -> Result<()> {
        self.check()?;
        self.roll_if_due(t_ns)?;
        if self.segment.is_none() {
            self.open_segment(t_ns)?;
        }
        let seg = self.segment.as_mut().expect("opened above");
        let channel_id = match seg.channels.get(topic) {
            Some(id) => *id,
            None => {
                let schema_id = match schema {
                    Some(name) => seg
                        .writer
                        .add_schema(name, "protobuf", FILE_DESCRIPTOR_SET)?,
                    None => 0,
                };
                let id = seg
                    .writer
                    .add_channel(schema_id, topic, encoding, &BTreeMap::new())?;
                seg.channels.insert(topic.to_string(), id);
                id
            }
        };
        let header = MessageHeader {
            channel_id,
            sequence: seg.seq,
            log_time: t_ns,
            publish_time: t_ns,
        };
        seg.writer.write_to_known_channel(&header, data)?;
        seg.seq = seg.seq.wrapping_add(1);
        seg.t_start_ns = seg.t_start_ns.min(t_ns);
        seg.t_end_ns = seg.t_end_ns.max(t_ns);
        let (add, last) = seg.step(t_ns, self.limits.max_duration_ns);
        seg.elapsed_ns = seg.elapsed_ns.saturating_add(add);
        seg.last_ns = last;
        self.check()
    }

    /// The latched sink error: once a segment write has failed, every write
    /// and the finish fail with it.
    fn check(&self) -> Result<()> {
        match self.fault.get() {
            Some((kind, msg)) => Err(Error::Io(std::io::Error::new(*kind, msg.clone()))),
            None => Ok(()),
        }
    }

    fn roll_if_due(&mut self, t_ns: u64) -> Result<()> {
        let due = self.segment.as_ref().is_some_and(|s| {
            s.high.load(Ordering::Relaxed) >= self.limits.max_bytes
                || s.elapsed_ns
                    .saturating_add(s.step(t_ns, self.limits.max_duration_ns).0)
                    >= self.limits.max_duration_ns
        });
        if due {
            self.close_segment()?;
        }
        Ok(())
    }

    fn open_segment(&mut self, t_ns: u64) -> Result<()> {
        let file = format!("segments/{:06}.mcap", self.next_index);
        self.next_index += 1;
        let high = Arc::new(AtomicU64::new(0));
        let inner = (self.open_sink)(&self.dir.join(&file))
            .inspect_err(|e| latch(&self.fault, e))?;
        let out = CountingWriter {
            inner,
            pos: 0,
            high: high.clone(),
            fault: self.fault.clone(),
        };
        let mut writer = mcap::WriteOptions::new()
            .compression(Some(mcap::Compression::Lz4))
            .profile(MCAP_PROFILE)
            .library(concat!("ados-capture-contract/", env!("CARGO_PKG_VERSION")))
            .create(out)?;
        let manifest_json = serde_json::to_vec_pretty(&self.manifest)?;
        for (name, data) in [
            (topics::RIG_ATTACHMENT, self.rig_json.as_slice()),
            (topics::MANIFEST_ATTACHMENT, manifest_json.as_slice()),
        ] {
            writer.attach(&mcap::Attachment {
                log_time: t_ns,
                create_time: t_ns,
                name: name.to_string(),
                media_type: "application/json".to_string(),
                data: data.into(),
            })?;
        }
        self.segment = Some(OpenSegment {
            writer,
            high,
            file,
            t_start_ns: t_ns,
            t_end_ns: t_ns,
            last_ns: t_ns,
            elapsed_ns: 0,
            channels: HashMap::new(),
            seq: 0,
        });
        self.check()
    }

    fn close_segment(&mut self) -> Result<()> {
        let Some(mut seg) = self.segment.take() else {
            return Ok(());
        };
        seg.writer.finish()?;
        let mut out = seg.writer.into_inner();
        out.flush()?;
        if self.fault.get().is_none() {
            out.inner.sync().inspect_err(|e| latch(&self.fault, e))?;
        }
        self.check()?;
        let bytes = fs::metadata(self.dir.join(&seg.file))?.len();
        self.manifest.segments.push(Segment {
            file: seg.file,
            t_start_ns: seg.t_start_ns,
            t_end_ns: seg.t_end_ns,
            bytes,
        });
        Ok(())
    }

    /// Close the open segment, write `manifest.json` and `checksums.b3`, and
    /// return the final manifest. A session whose segment sink failed is
    /// closed and reports the failure; it gets no manifest.
    pub fn finish(mut self) -> Result<Manifest> {
        self.close_segment()?;
        self.check()?;
        fs::write(
            self.dir.join("manifest.json"),
            serde_json::to_vec_pretty(&self.manifest)?,
        )?;
        write_checksums(&self.dir)?;
        Ok(self.manifest)
    }
}
