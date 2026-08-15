#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
ws_rate.py — measure the telemetry rate over the WebSocket, no browser, no deps.

sock_rate.py measures the C process. This measures what comes out the far side
of server.js, so running both tells you which hop is losing packets:

  on the BOT   ./ws_rate.py ws://localhost:8675      network excluded
  on the MAC   ./ws_rate.py ws://192.168.1.142:8675  network included

Same number from both: the network is fine. Lower from the Mac: it is the wifi.
Low from both: it is server.js.

Pure standard library -- no websockets package, no node. The client handshake
and frame parsing are simple enough to do directly, and needing to install
something is exactly the friction that stops you measuring.
"""

import argparse, base64, os, socket, sys, time, json
from urllib.parse import urlparse


def handshake(sock, host, port, path):
    key = base64.b64encode(os.urandom(16)).decode()
    req = (f"GET {path or '/'} HTTP/1.1\r\n"
           f"Host: {host}:{port}\r\n"
           "Upgrade: websocket\r\n"
           "Connection: Upgrade\r\n"
           f"Sec-WebSocket-Key: {key}\r\n"
           "Sec-WebSocket-Version: 13\r\n\r\n")
    sock.sendall(req.encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        d = sock.recv(4096)
        if not d:
            raise ConnectionError("server closed during handshake")
        buf += d
    head, _, rest = buf.partition(b"\r\n\r\n")
    if b"101" not in head.split(b"\r\n")[0]:
        raise ConnectionError("upgrade refused: " + head.split(b"\r\n")[0].decode())
    return rest


class Frames:
    """Minimal RFC6455 reader. Server->client frames are never masked."""

    def __init__(self, sock, initial=b""):
        self.s = sock
        self.buf = initial

    def _need(self, n):
        while len(self.buf) < n:
            d = self.s.recv(65536)
            if not d:
                raise ConnectionError("closed")
            self.buf += d

    def next(self):
        self._need(2)
        b0, b1 = self.buf[0], self.buf[1]
        opcode = b0 & 0x0F
        masked = b1 & 0x80
        ln = b1 & 0x7F
        off = 2
        if ln == 126:
            self._need(4); ln = int.from_bytes(self.buf[2:4], "big"); off = 4
        elif ln == 127:
            self._need(10); ln = int.from_bytes(self.buf[2:10], "big"); off = 10
        if masked:
            self._need(off + 4); off += 4
        self._need(off + ln)
        payload = self.buf[off:off + ln]
        self.buf = self.buf[off + ln:]
        return opcode, payload


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("url", nargs="?", default="ws://localhost:8675")
    ap.add_argument("seconds", nargs="?", type=float, default=5.0)
    a = ap.parse_args()

    u = urlparse(a.url)
    host, port = u.hostname or "localhost", u.port or 80
    print(f"  connecting to {host}:{port} …")

    s = socket.create_connection((host, port), timeout=5)
    try:
        rest = handshake(s, host, port, u.path)
    except Exception as e:
        print(f"  handshake failed: {e}")
        return 1
    print("  connected, sampling…")

    fr = Frames(s, rest)
    s.settimeout(3.0)
    n = bad = nbytes = 0
    gaps = []
    t0 = last = time.monotonic()

    try:
        while time.monotonic() - t0 < a.seconds:
            op, payload = fr.next()
            if op == 0x8:
                print("  server closed the connection")
                break
            if op == 0x9:      # ping -> ignore, we are only measuring
                continue
            if op not in (0x1, 0x2, 0x0):
                continue
            nbytes += len(payload)
            try:
                json.loads(payload.decode("utf-8", "replace"))
            except Exception:
                bad += 1
                continue
            now = time.monotonic()
            if n:
                gaps.append((now - last) * 1000.0)
            last = now
            n += 1
    except (socket.timeout, ConnectionError) as e:
        print(f"  stopped: {e}")
    finally:
        s.close()

    el = time.monotonic() - t0 or 1
    hz = n / el
    print(f"\n  {n} valid packets in {el:.1f}s = {hz:.1f} Hz")
    print(f"  {nbytes/1024:.1f} kB = {nbytes/el/1024:.1f} kB/s")
    if bad:
        print(f"  {bad} messages FAILED to parse  <- framing still broken")
    if gaps:
        gaps.sort()
        print(f"  gap: min {gaps[0]:.0f} ms, median {gaps[len(gaps)//2]:.0f} ms, "
              f"max {gaps[-1]:.0f} ms")
    print()
    if hz >= 8 and not bad:
        print("  This hop is healthy.")
    elif bad:
        print("  Arriving but not parsing: the framing is still wrong.")
    else:
        print("  This hop is LOSING packets. The fault is at or before it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
