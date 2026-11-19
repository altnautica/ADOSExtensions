//! Where one extension's processes find each other on one node.
//!
//! An extension may run as several processes under one plugin identity (a
//! main process plus declared services). They meet on Unix sockets in one
//! private directory: the per-plugin HTTP directory the plugin host creates
//! and binds read-write into every unit of an `agent.http` plugin (main and
//! services alike), announced through `ADOS_PLUGIN_HTTP_SOCKET`. No core
//! socket is used.
//!
//! Pure path arithmetic over an injected environment lookup, so the resolution
//! is unit-testable without touching the process environment.

use std::path::{Path, PathBuf};

/// The env var the host sets to the plugin's HTTP socket path.
pub const HTTP_SOCKET_ENV: &str = "ADOS_PLUGIN_HTTP_SOCKET";
/// The agent run-dir override, honoured when the HTTP socket env is absent.
pub const RUN_DIR_ENV: &str = "ADOS_RUN_DIR";

/// The HTTP socket file ados-control proxies `/api/plugins/<id>/x/*` to.
pub const HTTP_SOCKET: &str = "http.sock";

/// The private socket directory of `plugin_id`, from the process environment.
pub fn ipc_dir(plugin_id: &str) -> PathBuf {
    ipc_dir_from(plugin_id, |k| std::env::var(k).ok())
}

/// The private socket directory: the parent of `ADOS_PLUGIN_HTTP_SOCKET` when
/// the host set it, else `<ADOS_RUN_DIR or /run/ados>/plugin-http/<plugin id>`
/// (the same directory the host would have created).
pub fn ipc_dir_from(plugin_id: &str, env: impl Fn(&str) -> Option<String>) -> PathBuf {
    if let Some(sock) = env(HTTP_SOCKET_ENV) {
        if let Some(parent) = Path::new(&sock).parent() {
            if !parent.as_os_str().is_empty() {
                return parent.to_path_buf();
            }
        }
    }
    let run = env(RUN_DIR_ENV).unwrap_or_else(|| "/run/ados".to_string());
    Path::new(&run).join("plugin-http").join(plugin_id)
}

/// The HTTP socket path: `ADOS_PLUGIN_HTTP_SOCKET` verbatim when set, else
/// [`HTTP_SOCKET`] inside [`ipc_dir_from`].
pub fn http_socket_from(plugin_id: &str, env: impl Fn(&str) -> Option<String>) -> PathBuf {
    match env(HTTP_SOCKET_ENV) {
        Some(sock) if !sock.is_empty() => PathBuf::from(sock),
        _ => ipc_dir_from(plugin_id, env).join(HTTP_SOCKET),
    }
}

/// The HTTP socket path of `plugin_id` from the process environment.
pub fn http_socket(plugin_id: &str) -> PathBuf {
    http_socket_from(plugin_id, |k| std::env::var(k).ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_http_socket_directory_is_the_private_directory() {
        let env = |k: &str| {
            (k == HTTP_SOCKET_ENV).then(|| "/run/ados/plugin-http/com.x/http.sock".to_string())
        };
        assert_eq!(
            ipc_dir_from("com.y", env),
            PathBuf::from("/run/ados/plugin-http/com.x")
        );
        assert_eq!(
            http_socket_from("com.y", env),
            PathBuf::from("/run/ados/plugin-http/com.x/http.sock")
        );
    }

    #[test]
    fn without_the_host_env_the_run_dir_layout_is_derived() {
        let env = |k: &str| (k == RUN_DIR_ENV).then(|| "/tmp/run".to_string());
        assert_eq!(
            ipc_dir_from("com.example.capture", env),
            PathBuf::from("/tmp/run/plugin-http/com.example.capture")
        );
        assert_eq!(
            ipc_dir_from("com.example.capture", |_| None),
            PathBuf::from("/run/ados/plugin-http/com.example.capture")
        );
    }
}
