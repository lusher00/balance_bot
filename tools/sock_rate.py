#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
sock_rate.py — measure telemetry straight off the unix socket.

Connects to /tmp/balance_bot.sock as an extra client and counts newline-framed
packets. This bypasses server.js, the network and the browser entirely, so it
answers the only question that matters when the dashboard looks slow: is the
robot actually emitting at the rate it thinks it is?

  ./sock_rate.py            # 5 second sample
  ./sock_rate.py 15         # longer, to catch intermittent stalls

Run it on the bot.
"""
import socket, sys, time

PATH = "/tmp/balance_bot.sock"
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(PATH)
except OSError as e:
    print(f"  cannot connect to {PATH}: {e}")
    print("  is balance_bot running?")
    sys.exit(1)

s.settimeout(2.0)
buf = b""
n = 0
nbytes = 0
gaps = []
t0 = time.monotonic()
last = t0

while time.monotonic() - t0 < dur:
    try:
        d = s.recv(65536)
    except socket.timeout:
        break
    if not d:
        print("  socket closed by peer")
        break
    nbytes += len(d)
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        if not line.strip():
            continue
        now = time.monotonic()
        if n:
            gaps.append((now - last) * 1000.0)
        last = now
        n += 1

el = time.monotonic() - t0
s.close()

print(f"  {n} packets in {el:.1f}s = {n/el if el else 0:.1f} Hz")
print(f"  {nbytes/1024:.1f} kB total = {nbytes/el/1024 if el else 0:.1f} kB/s")
if gaps:
    gaps.sort()
    print(f"  gap between packets: min {gaps[0]:.0f} ms, "
          f"median {gaps[len(gaps)//2]:.0f} ms, max {gaps[-1]:.0f} ms")
print()
if n / (el or 1) >= 8:
    print("  Source is healthy. The firmware is emitting at rate, so anything")
    print("  slow downstream is server.js, the network, or the browser.")
elif n:
    print("  Source is SLOW. The firmware itself is not emitting at 10 Hz --")
    print("  nothing downstream can fix that.")
else:
    print("  No packets at all. Either nothing is broadcasting, or the packets")
    print("  carry no newline (pre-fix firmware).")
