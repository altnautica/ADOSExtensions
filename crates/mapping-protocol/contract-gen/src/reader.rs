//! [`SessionReader`]: reads a session directory written by any producer of
//! the contract.

use std::fs::{self, File};
use std::path::{Path, PathBuf};

use crate::schema::{Manifest, Rig};
use crate::{verify_checksums, Error, Result};

/// One recorded message, borrowed from the mapped segment.
#[derive(Debug, Clone, Copy)]
pub struct SessionMessage<'a> {
    pub topic: &'a str,
    /// Protobuf full name (`ados.capture.v1.ImuBatch`), or `None` for a
    /// schemaless channel.
    pub schema_name: Option<&'a str>,
    /// MCAP message encoding (`protobuf` or `json`).
    pub message_encoding: &'a str,
    /// Message time on the session timebase.
    pub t_ns: u64,
    pub data: &'a [u8],
}

impl SessionMessage<'_> {
    /// Decode as `M`, refusing a channel whose schema is not `M`.
    pub fn decode<M: prost::Message + prost::Name + Default>(&self) -> Result<M> {
        if self.schema_name != Some(M::full_name().as_str()) {
            return Err(Error::Invalid(format!(
                "{} carries {:?}, not {}",
                self.topic,
                self.schema_name,
                M::full_name()
            )));
        }
        Ok(M::decode(self.data)?)
    }

    /// Parse a JSON channel's message.
    pub fn json(&self) -> Result<serde_json::Value> {
        Ok(serde_json::from_slice(self.data)?)
    }
}

/// An opened session directory.
#[derive(Debug, Clone)]
pub struct SessionReader {
    dir: PathBuf,
    manifest: Manifest,
    rig: Rig,
}

impl SessionReader {
    /// Read `manifest.json` and `rig.json` from `dir`.
    pub fn open(dir: &Path) -> Result<Self> {
        let manifest: Manifest = serde_json::from_slice(&fs::read(dir.join("manifest.json"))?)?;
        let rig: Rig = serde_json::from_slice(&fs::read(dir.join("rig.json"))?)?;
        rig.check()?;
        for seg in &manifest.segments {
            let ok = seg.file.strip_prefix("segments/").is_some_and(|f| {
                f.len() == 11 && f.ends_with(".mcap") && f[..6].bytes().all(|b| b.is_ascii_digit())
            });
            if !ok {
                return Err(Error::Invalid(format!("bad segment path `{}`", seg.file)));
            }
        }
        Ok(Self {
            dir: dir.to_path_buf(),
            manifest,
            rig,
        })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    pub fn rig(&self) -> &Rig {
        &self.rig
    }

    /// Check every file against `checksums.b3`.
    pub fn verify(&self) -> Result<()> {
        verify_checksums(&self.dir)
    }

    /// Visit every message of every segment in manifest order (within a
    /// segment, in file order).
    pub fn for_each_message(
        &self,
        mut f: impl FnMut(SessionMessage<'_>) -> Result<()>,
    ) -> Result<()> {
        for seg in &self.manifest.segments {
            let file = File::open(self.dir.join(&seg.file))?;
            // SAFETY: segments are immutable once the manifest lists them; the
            // map is dropped before this function returns.
            let map = unsafe { memmap2::Mmap::map(&file)? };
            for msg in mcap::MessageStream::new(&map)? {
                let msg = msg?;
                f(SessionMessage {
                    topic: &msg.channel.topic,
                    schema_name: msg.channel.schema.as_ref().map(|s| s.name.as_str()),
                    message_encoding: &msg.channel.message_encoding,
                    t_ns: msg.log_time,
                    data: &msg.data,
                })?;
            }
        }
        Ok(())
    }
}
