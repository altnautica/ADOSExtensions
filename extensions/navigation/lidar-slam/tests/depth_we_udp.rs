//! The World Engine depth driver against a stand-in server on loopback: it
//! subscribes, rebuilds frames sent whole, reordered and with a lost
//! datagram, counts the drop, and keeps its subscription alive.

use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

use navigation_lidar_slam::depth_source::we::{WeConfig, WeSource};
use navigation_lidar_slam::depth_source::wire::{self, Header, SUBSCRIBE};
use navigation_lidar_slam::depth_source::{DepthSource, DepthState, Extrinsic};

const W: u16 = 160;
const H: u16 = 120;

fn frame(seq: u32) -> (Header, Vec<u16>) {
    let head = Header {
        seq,
        sim_ns: 5_000_000_000 + u64::from(seq) * 100_000_000,
        w: W,
        h: H,
        fx: 84.3024,
        fy: 108.2429,
        cx: 80.0,
        cy: 60.0,
        frag: 0,
        frags: 0,
        offset: 0,
    };
    // A wall 4 m ahead with a band of sky (no return) along the top rows.
    let mut depth = vec![4000u16; usize::from(W) * usize::from(H)];
    depth[..usize::from(W) * 10].fill(0);
    depth[usize::from(W) * 60 + 80] = 4000 + seq as u16;
    (head, depth)
}

fn datagrams(seq: u32) -> (Vec<u16>, Vec<Vec<u8>>) {
    let (head, depth) = frame(seq);
    let mut out = Vec::new();
    let mut buf = Vec::new();
    wire::for_each_datagram(&head, &depth, &mut buf, |d| out.push(d.to_vec()));
    (depth, out)
}

/// Wait for the driver's subscription; returns its address.
fn subscriber(server: &UdpSocket) -> SocketAddr {
    let mut b = [0u8; 64];
    let (n, from) = server.recv_from(&mut b).expect("no subscription");
    assert_eq!(&b[..n], SUBSCRIBE);
    from
}

#[test]
fn we_driver_rebuilds_frames_over_loopback_udp() {
    let server = UdpSocket::bind("127.0.0.1:0").unwrap();
    server
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let ext = Extrinsic::from_rpy_deg([0.13, 0.0, 0.0], [0.0, -20.0, 0.0]);
    let mut src = WeSource::open(&WeConfig {
        host: "127.0.0.1".into(),
        port: server.local_addr().unwrap().port(),
        extrinsic: ext,
    })
    .unwrap();
    let client = subscriber(&server);
    assert_eq!(client.port(), src.local_addr().unwrap().port());
    assert!(src.next_frame(Duration::from_millis(50)).unwrap().is_none());
    assert_eq!(src.status(1).state, DepthState::Waiting);

    // Frame 0 in order.
    let (d0, f0) = datagrams(0);
    for d in &f0 {
        server.send_to(d, client).unwrap();
    }
    let got = src
        .next_frame(Duration::from_secs(2))
        .unwrap()
        .expect("frame 0");
    assert_eq!(
        (got.seq, got.source_ns, got.w, got.h),
        (0, 5_000_000_000, 160, 120)
    );
    assert_eq!((got.fx, got.cy), (84.3024, 60.0));
    assert_eq!(got.depth_mm, d0);
    assert_eq!(got.extrinsic, ext);
    assert!(got.stamp_ns > 0);
    let mut pts = Vec::new();
    got.to_points(1, 40.0, &mut pts);
    assert_eq!(pts.len(), usize::from(W) * usize::from(H - 10));
    src.recycle(got);

    // Frame 1 loses a datagram; frame 2 arrives reversed.
    let (_, f1) = datagrams(1);
    for (i, d) in f1.iter().enumerate() {
        if i != 7 {
            server.send_to(d, client).unwrap();
        }
    }
    let (d2, mut f2) = datagrams(2);
    f2.reverse();
    for d in &f2 {
        server.send_to(d, client).unwrap();
    }
    let got = src
        .next_frame(Duration::from_secs(2))
        .unwrap()
        .expect("frame 2");
    assert_eq!(got.seq, 2);
    assert_eq!(got.depth_mm, d2);
    let h = src.health();
    assert_eq!((h.frames, h.drops, h.rejected), (2, 1, 0));
    assert_eq!(h.last_stamp_ns, got.stamp_ns);
    assert_eq!(src.status(got.stamp_ns).state, DepthState::Streaming);

    // Garbage on the port is counted, not fatal.
    server.send_to(b"not a depth frame", client).unwrap();
    assert!(src
        .next_frame(Duration::from_millis(100))
        .unwrap()
        .is_none());
    assert_eq!(src.health().rejected, 1);

    // The subscription is renewed while the driver waits.
    let renew = std::thread::spawn(move || {
        let from = subscriber(&server);
        (server, from)
    });
    assert!(src
        .next_frame(Duration::from_millis(1300))
        .unwrap()
        .is_none());
    let (_server, from) = renew.join().unwrap();
    assert_eq!(from, client);
}
