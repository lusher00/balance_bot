#!/usr/bin/env python3
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# This file is part of balance_bot, licensed under the PolyForm
# Noncommercial License 1.0.0. You may use, study, modify, and share
# it for any noncommercial purpose. Commercial use requires a separate
# license from the author -- contact ryan.lush@gmail.com.
# Full license text: see the LICENSE file in the project root, or
# https://polyformproject.org/licenses/noncommercial/1.0.0/

"""
sock_rate.py — measure telemetry straight off the unix socket, per message type.

Connects to /tmp/balance_bot.sock as an extra client and counts newline-framed
packets. This bypasses server.js, the network and the browser entirely, so it
answers the only question that matters when the dashboard looks slow: is the
robot actually emitting at the rate it thinks it is?

The stream carries three message types (telemetry / rc / config), and they are
reported separately on purpose. A single blended packet rate hides the failure
that matters: RC arriving happily at 30 Hz while telemetry is being starved is
what puts straight angular segments in the graphs, and a blended number looks
fine right through it.

  ./sock_rate.py            # 5 second sample
  ./sock_rate.py 20         # longer, to catch intermittent stalls

The transmitter test:

  ./sock_rate.py 20         # transmitter OFF
  ./sock_rate.py 20         # transmitter ON

If telemetry stays even in both, the bot is fine and the fault is downstream
(server.js, wifi, or the browser). If its gaps blow up only with the
transmitter on, the fault is on the bot and this says so.

Run it on the bot.
"""
import socket, sys, time, json

PATH = "/tmp/balance_bot.sock"
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


def pct(vals, q):
    if not vals:
        return 0.0
    v = sorted(vals)
    return v[min(len(v) - 1, int(q * (len(v) - 1)))]


try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(PATH)
except OSError as e:
    print(f"  cannot connect to {PATH}: {e}")
    print("  is balance_bot running?")
    sys.exit(1)

s.settimeout(2.0)
buf = b""
stat = {}                       # type -> {"n", "bytes", "gaps", "last"}
loops = []                      # loop_hz samples from telemetry
drops_first = drops_last = None
bad = 0                         # lines that would not parse

print(f"  sampling {dur:.0f} s from {PATH} ...")
t0 = time.monotonic()

while time.monotonic() - t0 < dur:
    try:
        d = s.recv(65536)
    except socket.timeout:
        continue
    if not d:
        print("  socket closed by balance_bot")
        break
    now = time.monotonic()
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        if not line.strip():
            continue
        try:
            o = json.loads(line)
        except Exception:
            bad += 1
            continue
        t = o.get("type", "?")
        st = stat.setdefault(t, {"n": 0, "bytes": 0, "gaps": [], "last": None})
        st["n"] += 1
        st["bytes"] += len(line) + 1
        if st["last"] is not None:
            st["gaps"].append((now - st["last"]) * 1000.0)
        st["last"] = now
        if t == "telemetry":
            sysd = o.get("system") or {}
            if "loop_hz" in sysd:
                loops.append(sysd["loop_hz"])
            if "tx_drops" in sysd:
                global_d = sysd["tx_drops"]
                drops_first = global_d if drops_first is None else drops_first
                drops_last = global_d

el = time.monotonic() - t0
s.close()

print()
print(f"  {'type':<11}{'pkts':>7}{'rate':>10}{'gap p50':>10}{'gap p95':>10}{'gap max':>10}")
total_bytes = 0
for t in ("telemetry", "rc", "config"):
    st = stat.get(t)
    if not st:
        print(f"  {t:<11}{0:>7}{'--':>10}{'--':>10}{'--':>10}{'--':>10}")
        continue
    total_bytes += st["bytes"]
    g = st["gaps"]
    if len(g) < 2:
        print(f"  {t:<11}{st['n']:>7}{'(one-shot)':>10}{'--':>10}{'--':>10}{'--':>10}")
        continue
    print(f"  {t:<11}{st['n']:>7}{st['n']/el:>9.1f}H"
          f"{pct(g,0.5):>9.0f}m{pct(g,0.95):>9.0f}m{max(g):>9.0f}m")
for t in stat:
    if t not in ("telemetry", "rc", "config"):
        print(f"  {t:<11}{stat[t]['n']:>7}   (unrecognised type)")

print()
if loops:
    print(f"  loop_hz     min {min(loops):.1f}   mean {sum(loops)/len(loops):.1f}   max {max(loops):.1f}")
if drops_last is not None:
    delta = drops_last - drops_first
    print(f"  tx_drops    {drops_last}  ({'+%d during this sample' % delta if delta else 'unchanged'})")
print(f"  throughput  {total_bytes/el/1024:.1f} KB/s")
if bad:
    print(f"  UNPARSEABLE LINES: {bad}  <- stream framing is broken")

# ── verdict ──────────────────────────────────────────────────────────────────
print()
tel = stat.get("telemetry")
if not tel or len(tel["gaps"]) < 5:
    print("  not enough telemetry to judge — is the loop running?")
    sys.exit(0)

g = tel["gaps"]
nominal = pct(g, 0.5)
worst = max(g)
ratio = worst / nominal if nominal else 0

if ratio < 2.0:
    print(f"  VERDICT: telemetry spacing is even (worst gap {worst:.0f} ms vs {nominal:.0f} ms typical).")
    print("           The bot is emitting cleanly — if the graphs still look jagged,")
    print("           the loss is downstream: server.js, the wifi, or the browser.")
    print("           Compare with tools/ws_rate.py to find which hop.")
else:
    missed = int(round(worst / nominal)) - 1
    print(f"  VERDICT: telemetry is STALLING on the bot. Worst gap {worst:.0f} ms against")
    print(f"           {nominal:.0f} ms typical — about {missed} sample(s) missing in a row.")
    print("           That is a straight angular segment in the graph, and it happens")
    print("           before any of this reaches server.js or the browser.")
    if loops and min(loops) < 90:
        print(f"           loop_hz fell to {min(loops):.0f} — the control loop is late, which")
        print("           delays telemetry because the loop is what clocks it.")
    if drops_last:
        print(f"           tx_drops is {drops_last} — the socket buffer filled, so a client")
        print("           was not draining fast enough (that is server.js or the link).")
