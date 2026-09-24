//! The credentials this workstation issues other nodes.
//!
//! A drone reaches this node's TCP listener only with a credential the operator
//! had this node issue (over the operator socket), scoped to the lanes a drone
//! uses and installed on the drone (see [`ados_protocol::node_credential`]).
//! The lane gate in [`crate::auth`] then admits that credential for exactly
//! those lanes.
//!
//! Each issued record keeps only a SHA-256 digest of the secret. Issuing again
//! for the same peer replaces its earlier credential, so a re-provision is also
//! a rotation; revoking removes it. The store is persisted owner-only beside
//! the job store.

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use ados_protocol::node_credential::NodeLane;
use ados_protocol::pairing_posture::constant_time_eq;

/// Default location of the issued-credential store.
pub const DEFAULT_NODE_CREDENTIALS_PATH: &str = "/var/ados/compute/node-credentials.json";

/// The version prefix of an issued token: `nc1.<id>.<secret>`.
const TOKEN_PREFIX: &str = "nc1";
/// Id and secret lengths, drawn from the unambiguous charset (31 symbols): the
/// secret carries over 200 bits.
const ID_LEN: usize = 12;
const SECRET_LEN: usize = 43;
/// A peer device id is a node id, not free text.
const MAX_PEER_ID_LEN: usize = 128;

/// A failure issuing or revoking a credential.
#[derive(Debug, thiserror::Error)]
pub enum CredentialError {
    #[error("peer device id must be 1-{MAX_PEER_ID_LEN} characters")]
    BadPeer,
    #[error("at least one lane is required")]
    NoLanes,
    #[error("entropy source failed: {0}")]
    Entropy(String),
    #[error("persist node credentials: {0}")]
    Persist(#[from] std::io::Error),
}

/// One issued credential as stored (never the secret itself).
#[derive(Debug, Clone, Serialize, Deserialize)]
struct IssuedRecord {
    id: String,
    peer_device_id: String,
    lanes: Vec<NodeLane>,
    created_at_ms: i64,
    secret_sha256: String,
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct StoreFile {
    #[serde(default)]
    credentials: Vec<IssuedRecord>,
}

/// An issued credential as the owner sees it in a listing.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct IssuedCredential {
    pub id: String,
    pub peer_device_id: String,
    pub lanes: Vec<NodeLane>,
    pub created_at_ms: i64,
    /// Whether the credential admits its lanes. Every listed credential does
    /// (a revoked one is removed), so this is always `true`.
    pub current: bool,
}

/// A freshly issued credential, carrying the secret token exactly once.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct MintedCredential {
    pub id: String,
    pub peer_device_id: String,
    pub lanes: Vec<NodeLane>,
    pub created_at_ms: i64,
    /// The token the peer presents in the node-credential header.
    pub credential: String,
    /// This node's id, the one it advertises over mDNS: the peer files the
    /// credential under it so each lane presents it only to this node.
    pub workstation_node_id: String,
}

/// The persisted set of credentials this node issued.
pub struct NodeCredentialStore {
    path: PathBuf,
    node_id: String,
    records: Mutex<Vec<IssuedRecord>>,
}

impl NodeCredentialStore {
    /// Open the store at `path` for the node `node_id`. An absent file is an
    /// empty store. An unreadable or corrupt one is also opened empty (and
    /// logged): nothing it held can be verified, which refuses rather than
    /// admits, and the owner re-provisions.
    pub fn open(path: PathBuf, node_id: impl Into<String>) -> Self {
        let records = match std::fs::read(&path) {
            Ok(bytes) => match serde_json::from_slice::<StoreFile>(&bytes) {
                Ok(file) => file.credentials,
                Err(e) => {
                    tracing::warn!(path = %path.display(), error = %e, "node credential store unreadable; starting empty");
                    Vec::new()
                }
            },
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
            Err(e) => {
                tracing::warn!(path = %path.display(), error = %e, "node credential store unreadable; starting empty");
                Vec::new()
            }
        };
        Self {
            path,
            node_id: node_id.into(),
            records: Mutex::new(records),
        }
    }

    /// This node's id (the issuer named in a mint reply).
    pub fn node_id(&self) -> &str {
        &self.node_id
    }

    /// Issue a credential for `peer_device_id` scoped to `lanes`, replacing any
    /// earlier credential for that peer. Nothing is admitted unless the store
    /// persisted.
    pub fn mint(
        &self,
        peer_device_id: &str,
        lanes: &[NodeLane],
        now_ms: i64,
    ) -> Result<MintedCredential, CredentialError> {
        let peer = peer_device_id.trim();
        if peer.is_empty() || peer.len() > MAX_PEER_ID_LEN {
            return Err(CredentialError::BadPeer);
        }
        let mut lanes: Vec<NodeLane> = lanes.to_vec();
        lanes.sort();
        lanes.dedup();
        if lanes.is_empty() {
            return Err(CredentialError::NoLanes);
        }
        let id = ados_protocol::secret_gen::generate(ID_LEN)
            .map_err(|e| CredentialError::Entropy(e.to_string()))?;
        let secret = ados_protocol::secret_gen::generate(SECRET_LEN)
            .map_err(|e| CredentialError::Entropy(e.to_string()))?;
        let record = IssuedRecord {
            id: id.clone(),
            peer_device_id: peer.to_string(),
            lanes: lanes.clone(),
            created_at_ms: now_ms,
            secret_sha256: digest(&secret),
        };
        let mut records = self.records.lock().unwrap_or_else(|p| p.into_inner());
        let mut next: Vec<IssuedRecord> = records
            .iter()
            .filter(|r| r.peer_device_id != peer)
            .cloned()
            .collect();
        next.push(record);
        persist(&self.path, &next)?;
        *records = next;
        Ok(MintedCredential {
            credential: format!("{TOKEN_PREFIX}.{id}.{secret}"),
            id,
            peer_device_id: peer.to_string(),
            lanes,
            created_at_ms: now_ms,
            workstation_node_id: self.node_id.clone(),
        })
    }

    /// Every issued credential (no secrets).
    pub fn list(&self) -> Vec<IssuedCredential> {
        let records = self.records.lock().unwrap_or_else(|p| p.into_inner());
        records
            .iter()
            .map(|r| IssuedCredential {
                id: r.id.clone(),
                peer_device_id: r.peer_device_id.clone(),
                lanes: r.lanes.clone(),
                created_at_ms: r.created_at_ms,
                current: true,
            })
            .collect()
    }

    /// Revoke credential `id`. Returns whether one was removed. The revocation
    /// takes effect in memory even if persisting it fails; the error reports
    /// that it would not survive a restart.
    pub fn revoke(&self, id: &str) -> Result<bool, CredentialError> {
        let mut records = self.records.lock().unwrap_or_else(|p| p.into_inner());
        let before = records.len();
        records.retain(|r| r.id != id);
        if records.len() == before {
            return Ok(false);
        }
        persist(&self.path, &records)?;
        Ok(true)
    }

    /// Whether `token` is a live credential for `lane`.
    pub fn admits(&self, token: &str, lane: &NodeLane) -> bool {
        let Some((id, secret)) = parse_token(token) else {
            return false;
        };
        let records = self.records.lock().unwrap_or_else(|p| p.into_inner());
        let Some(record) = records.iter().find(|r| r.id == id) else {
            return false;
        };
        record.lanes.contains(lane)
            && constant_time_eq(record.secret_sha256.as_bytes(), digest(secret).as_bytes())
    }
}

/// Split `nc1.<id>.<secret>`.
fn parse_token(token: &str) -> Option<(&str, &str)> {
    let mut parts = token.trim().splitn(3, '.');
    if parts.next()? != TOKEN_PREFIX {
        return None;
    }
    let id = parts.next()?;
    let secret = parts.next()?;
    (!id.is_empty() && !secret.is_empty()).then_some((id, secret))
}

fn digest(secret: &str) -> String {
    hex::encode(Sha256::digest(secret.as_bytes()))
}

/// Atomic, owner-only write (temp sibling, fsync, rename).
fn persist(path: &Path, records: &[IssuedRecord]) -> std::io::Result<()> {
    let body = serde_json::to_vec_pretty(&StoreFile {
        credentials: records.to_vec(),
    })
    .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let tmp = path.with_extension("json.tmp");
    let mut opts = std::fs::OpenOptions::new();
    opts.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt as _;
        opts.mode(0o600);
    }
    let mut f = opts.open(&tmp)?;
    f.write_all(&body)?;
    f.sync_all()?;
    drop(f);
    std::fs::rename(&tmp, path).inspect_err(|_| {
        let _ = std::fs::remove_file(&tmp);
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store(dir: &Path) -> NodeCredentialStore {
        NodeCredentialStore::open(dir.join("creds.json"), "ws-node")
    }

    #[test]
    fn a_minted_credential_admits_only_its_lanes() {
        let dir = tempfile::tempdir().unwrap();
        let s = store(dir.path());
        let m = s
            .mint("drone-1", &[crate::lanes::ATLAS_INGEST], 10)
            .unwrap();
        assert_eq!(m.workstation_node_id, "ws-node");
        assert!(s.admits(&m.credential, &crate::lanes::ATLAS_INGEST));
        // Another lane is refused.
        assert!(!s.admits(&m.credential, &crate::lanes::JOB_SUBMIT));
        // A tampered secret is refused.
        let forged = format!("{}x", m.credential);
        assert!(!s.admits(&forged, &crate::lanes::ATLAS_INGEST));
        assert!(!s.admits("", &crate::lanes::ATLAS_INGEST));
        assert!(!s.admits("nc1.nope.nope", &crate::lanes::ATLAS_INGEST));
    }

    #[test]
    fn reissuing_for_a_peer_rotates_its_credential() {
        let dir = tempfile::tempdir().unwrap();
        let s = store(dir.path());
        let first = s.mint("drone-1", &crate::lanes::ALL, 1).unwrap();
        let second = s.mint("drone-1", &crate::lanes::ALL, 2).unwrap();
        assert!(!s.admits(&first.credential, &crate::lanes::ATLAS_INGEST));
        assert!(s.admits(&second.credential, &crate::lanes::ATLAS_INGEST));
        assert_eq!(s.list().len(), 1);
    }

    #[test]
    fn revoke_refuses_the_credential_and_survives_a_restart() {
        let dir = tempfile::tempdir().unwrap();
        let s = store(dir.path());
        let a = s.mint("drone-a", &crate::lanes::ALL, 1).unwrap();
        let b = s.mint("drone-b", &crate::lanes::ALL, 1).unwrap();
        assert!(s.revoke(&a.id).unwrap());
        assert!(!s.revoke(&a.id).unwrap());
        assert!(!s.admits(&a.credential, &crate::lanes::ATLAS_INGEST));

        let reopened = store(dir.path());
        assert!(!reopened.admits(&a.credential, &crate::lanes::ATLAS_INGEST));
        assert!(reopened.admits(&b.credential, &crate::lanes::ATLAS_INGEST));
        let listed = reopened.list();
        assert_eq!(listed.len(), 1);
        assert_eq!(listed[0].peer_device_id, "drone-b");
    }

    #[test]
    fn the_store_keeps_no_secret_and_is_owner_only() {
        let dir = tempfile::tempdir().unwrap();
        let s = store(dir.path());
        let m = s.mint("drone-1", &crate::lanes::ALL, 1).unwrap();
        let path = dir.path().join("creds.json");
        let on_disk = std::fs::read_to_string(&path).unwrap();
        let secret = m.credential.rsplit('.').next().unwrap();
        assert!(!on_disk.contains(secret));
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600);
        }
    }

    #[test]
    fn bad_mint_requests_are_refused() {
        let dir = tempfile::tempdir().unwrap();
        let s = store(dir.path());
        assert!(matches!(
            s.mint(" ", &crate::lanes::ALL, 1),
            Err(CredentialError::BadPeer)
        ));
        assert!(matches!(
            s.mint("drone", &[], 1),
            Err(CredentialError::NoLanes)
        ));
    }
}
