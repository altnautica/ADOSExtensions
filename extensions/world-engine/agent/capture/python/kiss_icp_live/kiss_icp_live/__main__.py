from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

from . import serve
from .framing import FramingError
from .odometry import LiveOdometry, load_config


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="kiss_icp_live")
    ap.add_argument("--socket", required=True, help="Unix socket the capture service listens on")
    ap.add_argument("--config", type=Path, help="JSON: KISS-ICP config fields plus an optional 'live' section")
    args = ap.parse_args(argv)

    cfg, live = load_config(args.config)
    odom = LiveOdometry(cfg, live)
    if not hasattr(socket, "AF_UNIX"):
        print("kiss_icp_live: this platform has no Unix sockets", file=sys.stderr)
        return 2
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(args.socket)
    with sock, sock.makefile("rb") as rfile, sock.makefile("wb") as wfile:
        try:
            n = serve(rfile, wfile, odom)
        except FramingError as e:
            print(f"kiss_icp_live: {e}", file=sys.stderr)
            return 2
        except BrokenPipeError:
            return 0
    print(f"kiss_icp_live: stream closed after {n} scans", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
