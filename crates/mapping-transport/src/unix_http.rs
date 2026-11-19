//! Serve an axum router on a Unix socket: the plugin's `http.sock` (which
//! ados-control proxies `/api/plugins/<id>/x/*` to), bound with
//! `ados_sdk::http::bind`.
//!
//! `axum::serve` only accepts a TCP listener, so this drives hyper directly,
//! one task per accepted connection, with upgrades carried so a WebSocket route
//! works over the socket exactly as over TCP.

use std::convert::Infallible;
use std::future::Future;

use axum::body::Body;
use axum::Router;
use hyper::body::Incoming;
use hyper_util::rt::{TokioExecutor, TokioIo};
use hyper_util::server::conn::auto::Builder as ConnBuilder;
use tokio::net::{UnixListener, UnixStream};
use tower::Service;

/// Marker inserted into every request that arrived over a Unix socket, so a
/// handler or middleware can tell a local-socket caller from a TCP peer. On the
/// plugin's `http.sock` the caller is ados-control, which has already
/// authenticated the operator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnixPeer;

/// Accept connections on `listener` and serve `app` on each until `shutdown`
/// resolves. Connections already accepted run to completion on their own tasks.
pub async fn serve_unix<F>(listener: UnixListener, app: Router, shutdown: F)
where
    F: Future<Output = ()>,
{
    tokio::pin!(shutdown);
    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            accepted = listener.accept() => match accepted {
                Ok((stream, _)) => {
                    tokio::spawn(serve_conn(stream, app.clone()));
                }
                Err(e) => {
                    tracing::warn!(error = %e, "unix socket accept failed");
                    tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                }
            },
        }
    }
}

/// Drive one accepted connection through hyper with the axum service.
async fn serve_conn(stream: UnixStream, app: Router) {
    let svc = hyper::service::service_fn(move |req: hyper::Request<Incoming>| {
        let mut app = app.clone();
        async move {
            let mut req = req.map(Body::new);
            req.extensions_mut().insert(UnixPeer);
            let response = app.call(req).await?;
            Ok::<_, Infallible>(response)
        }
    });
    if let Err(e) = ConnBuilder::new(TokioExecutor::new())
        .serve_connection_with_upgrades(TokioIo::new(stream), svc)
        .await
    {
        tracing::debug!(error = %e, "unix socket connection ended");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::http::request::Parts;
    use axum::routing::get;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    #[tokio::test]
    async fn a_request_over_the_socket_reaches_the_router_marked_as_local() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("http.sock");
        let listener = UnixListener::bind(&path).unwrap();

        let app = Router::new().route(
            "/ping",
            get(|parts: Parts| async move {
                if parts.extensions.get::<UnixPeer>().is_some() {
                    "local"
                } else {
                    "remote"
                }
            }),
        );
        let (stop_tx, stop_rx) = tokio::sync::oneshot::channel::<()>();
        let server = tokio::spawn(serve_unix(listener, app, async {
            let _ = stop_rx.await;
        }));

        let mut conn = UnixStream::connect(&path).await.unwrap();
        conn.write_all(b"GET /ping HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n")
            .await
            .unwrap();
        let mut reply = String::new();
        conn.read_to_string(&mut reply).await.unwrap();
        assert!(reply.starts_with("HTTP/1.1 200"), "{reply}");
        assert!(reply.ends_with("local"), "{reply}");

        let _ = stop_tx.send(());
        server.await.unwrap();
    }
}
