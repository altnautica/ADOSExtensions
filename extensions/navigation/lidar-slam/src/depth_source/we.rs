//! The World Engine depth stream: subscribes to a vehicle's depth port
//! (`7810 + SITL lease` on the World Engine host) and rebuilds its
//! `ADOSDEPTH1` frames ([`super::wire`]).
//!
//! The subscription is a [`SUBSCRIBE`] datagram from this driver's socket,
//! sent at open and renewed every [`RESUBSCRIBE`] (the World Engine forgets
//! a subscriber not heard for 5 s), so frames come back to the same socket.
//! Only datagrams from the stream's port are read. A World Engine run
//! without a renderer serves nothing: the driver then opens and waits.

use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::time::{Duration, Instant};

use super::wire::{Reassembler, MAX_DATAGRAM, SUBSCRIBE};
use super::{DepthFrame, DepthHealth, DepthSource, DepthSourceError, DepthSourceKind, Extrinsic};

/// Interval between subscription renewals.
pub const RESUBSCRIBE: Duration = Duration::from_secs(1);

#[derive(Debug, Clone)]
pub struct WeConfig {
    pub host: String,
    pub port: u16,
    pub extrinsic: Extrinsic,
}

pub struct WeSource {
    socket: UdpSocket,
    server: SocketAddr,
    last_sub: Instant,
    asm: Reassembler,
    buf: Vec<u8>,
    extrinsic: Extrinsic,
    last_seq: Option<u32>,
    health: DepthHealth,
}

impl WeSource {
    /// Resolve the stream's address, bind an ephemeral UDP port and
    /// subscribe.
    pub fn open(cfg: &WeConfig) -> Result<Self, DepthSourceError> {
        let server = (cfg.host.as_str(), cfg.port)
            .to_socket_addrs()
            .map_err(|e| {
                DepthSourceError::Unavailable(format!("cannot resolve {}: {e}", cfg.host))
            })?
            .min_by_key(|a| a.is_ipv6())
            .ok_or_else(|| {
                DepthSourceError::Unavailable(format!("{} resolves to no address", cfg.host))
            })?;
        let bind: SocketAddr = if server.is_ipv6() {
            "[::]:0"
                .parse()
                .map_err(|_| DepthSourceError::Stream("bind address".into()))?
        } else {
            "0.0.0.0:0"
                .parse()
                .map_err(|_| DepthSourceError::Stream("bind address".into()))?
        };
        let socket = UdpSocket::bind(bind)?;
        socket.send_to(SUBSCRIBE, server)?;
        Ok(Self {
            socket,
            server,
            last_sub: Instant::now(),
            asm: Reassembler::default(),
            buf: vec![0; MAX_DATAGRAM + 64],
            extrinsic: cfg.extrinsic,
            last_seq: None,
            health: DepthHealth::default(),
        })
    }

    /// The local address frames arrive on.
    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    fn renew(&mut self, now: Instant) -> std::io::Result<()> {
        if now.duration_since(self.last_sub) >= RESUBSCRIBE {
            self.last_sub = now;
            self.socket.send_to(SUBSCRIBE, self.server)?;
        }
        Ok(())
    }

    fn frame(&mut self, a: super::wire::Assembled) -> DepthFrame {
        let seq = a.head.seq;
        if let Some(prev) = self.last_seq {
            let gap = seq.wrapping_sub(prev);
            if gap > 1 && gap < super::wire::RESTART_GAP {
                self.health.drops += u64::from(gap - 1);
            }
        }
        self.last_seq = Some(seq);
        let stamp_ns = ados_mapping_pose::clock::mono_ns();
        self.health.frames += 1;
        self.health.last_stamp_ns = stamp_ns;
        DepthFrame {
            stamp_ns,
            source_ns: a.head.sim_ns,
            seq: u64::from(seq),
            w: u32::from(a.head.w),
            h: u32::from(a.head.h),
            fx: a.head.fx,
            fy: a.head.fy,
            cx: a.head.cx,
            cy: a.head.cy,
            depth_mm: a.depth_mm,
            extrinsic: self.extrinsic,
        }
    }
}

impl DepthSource for WeSource {
    fn kind(&self) -> DepthSourceKind {
        DepthSourceKind::We
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>, DepthSourceError> {
        let deadline = Instant::now() + timeout;
        loop {
            let now = Instant::now();
            // A send error (host unreachable for now) is retried at the next
            // renewal; the wait for frames goes on.
            let _ = self.renew(now);
            let Some(left) = deadline
                .checked_duration_since(now)
                .filter(|d| !d.is_zero())
            else {
                return Ok(None);
            };
            let until_renew = RESUBSCRIBE.saturating_sub(now.duration_since(self.last_sub));
            let wait = left.min(until_renew).max(Duration::from_millis(1));
            self.socket.set_read_timeout(Some(wait))?;
            match self.socket.recv_from(&mut self.buf) {
                Ok((n, from)) => {
                    if from.port() != self.server.port() {
                        continue;
                    }
                    if let Some(a) = self.asm.push(&self.buf[..n]) {
                        self.health.rejected = self.asm.stats.rejected;
                        return Ok(Some(self.frame(a)));
                    }
                    self.health.rejected = self.asm.stats.rejected;
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock
                            | std::io::ErrorKind::TimedOut
                            | std::io::ErrorKind::Interrupted
                            // An ICMP port-unreachable from a renewal sent
                            // before the World Engine bound its port.
                            | std::io::ErrorKind::ConnectionRefused
                    ) => {}
                Err(e) => return Err(DepthSourceError::Stream(e.to_string())),
            }
        }
    }

    fn health(&self) -> DepthHealth {
        self.health
    }

    fn recycle(&mut self, frame: DepthFrame) {
        self.asm.recycle(frame.depth_mm);
    }
}
