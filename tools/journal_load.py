#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
journal_load.py — what is writing to the SD card, and how fast?

Run on the bot. journald on this board is PERSISTENT (/var/log/journal exists),
so every log line is an SD-card write. A process logging in a 100 Hz loop is
therefore a 100 Hz writer to the card -- enough to make the board unresponsive
and, sustained, to fail a write and trip ext4 errors=remount-ro.

    ./tools/journal_load.py            # per-boot volume, then the noisiest boot
    ./tools/journal_load.py -1         # dig into a specific boot
    ./tools/journal_load.py --top 30   # more repeated-message detail

Note on `journalctl -b`: that is the CURRENT boot. After a crash-and-reboot,
with services stopped, it is empty by construction and proves nothing. The boot
you want is -1 (or -2). This defaults to scanning several and telling you which
one is abnormal, so it is hard to check the wrong one by accident.
"""
import argparse, os, re, subprocess, sys
from collections import Counter
from datetime import datetime


def run(args):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=180)
        return p.stdout
    except Exception as e:
        print(f"  ({' '.join(args)} failed: {e})", file=sys.stderr)
        return ""


def disk_usage():
    out = run(["journalctl", "--disk-usage"])
    return out.strip() or "unknown"


def boot_list(n=6):
    """Boot indices that actually exist, newest first: 0, -1, -2 ..."""
    out = run(["journalctl", "--list-boots", "--no-pager"])
    idx = []
    for line in out.splitlines():
        m = re.match(r"\s*(-?\d+)\s", line)
        if m:
            idx.append(int(m.group(1)))
    if not idx:
        idx = list(range(0, -n, -1))
    return sorted(idx, reverse=True)[:n]


def boot_lines(b):
    return run(["journalctl", "-b", str(b), "--no-pager", "-o", "short-iso"]).splitlines()


# strip the leading "2026-08-16T03:12:44+0000 hostname unit[pid]: " prefix so
# identical messages collapse together regardless of timestamp or pid
PREFIX = re.compile(r"^\S+\s+\S+\s+([^:\[]+)(\[\d+\])?:\s*(.*)$")
NUM = re.compile(r"-?\d+\.?\d*")


def normalise(line):
    m = PREFIX.match(line)
    unit, _, msg = m.groups() if m else ("?", None, line)
    # collapse varying numbers so "failed (-110)" and "failed (-9)" group
    return unit.strip(), NUM.sub("N", msg).strip()


def span_seconds(lines):
    ts = []
    for l in (lines[0], lines[-1]) if len(lines) > 1 else []:
        m = re.match(r"^(\S+)", l)
        if not m:
            continue
        try:
            ts.append(datetime.fromisoformat(m.group(1).replace("Z", "+00:00")))
        except ValueError:
            pass
    return (ts[1] - ts[0]).total_seconds() if len(ts) == 2 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("boot", nargs="?", type=int, default=None,
                    help="boot index to analyse (0 = current, -1 = previous)")
    ap.add_argument("--top", type=int, default=15, help="repeated messages to show")
    a = ap.parse_args()

    print(f"  journal on disk: {disk_usage()}")
    persistent = os.path.isdir("/var/log/journal")
    print(f"  storage: {'PERSISTENT — every line is an SD-card write' if persistent else 'volatile (tmpfs) — logging cannot wear the card'}")
    print()

    boots = boot_list()
    print(f"  {'boot':>6}{'lines':>10}{'seconds':>10}{'lines/s':>10}")
    print("  " + "-" * 36)
    stats = {}
    for b in boots:
        lines = boot_lines(b)
        if not lines:
            continue
        sec = span_seconds(lines)
        rate = len(lines) / sec if sec > 0 else 0.0
        stats[b] = (lines, sec, rate)
        flag = "   <-- abnormal" if rate > 5 else ""
        print(f"  {b:>6}{len(lines):>10}{sec:>10.0f}{rate:>10.1f}{flag}")

    if not stats:
        print("\n  no journal data at all — is journald running?")
        return 1

    target = a.boot
    if target is None:
        # the noisiest boot by rate, which is the one that hurt the card
        target = max(stats, key=lambda b: stats[b][2])
        print(f"\n  analysing boot {target} (highest line rate)")
    elif target not in stats:
        print(f"\n  boot {target} has no data")
        return 1
    else:
        print(f"\n  analysing boot {target}")

    lines, sec, rate = stats[target]
    print()

    by_unit = Counter()
    by_msg = Counter()
    for l in lines:
        unit, msg = normalise(l)
        by_unit[unit] += 1
        by_msg[(unit, msg)] += 1

    print("  which unit wrote the most:")
    for unit, n in by_unit.most_common(8):
        r = n / sec if sec > 0 else 0
        print(f"    {unit[:38]:<40}{n:>8}  ({r:>6.1f}/s)")

    print()
    print(f"  most repeated messages (numbers collapsed to N):")
    for (unit, msg), n in by_msg.most_common(a.top):
        r = n / sec if sec > 0 else 0
        flag = "  <--" if r > 2 else ""
        print(f"    {n:>7}  ({r:>6.1f}/s)  {unit[:18]:<20}{msg[:64]}{flag}")

    print()
    hot = [(n / sec if sec > 0 else 0, u, m) for (u, m), n in by_msg.items()]
    hot = [h for h in hot if h[0] > 2]
    if hot:
        worst = max(hot)
        print(f"  VERDICT: {len(hot)} message(s) are being written more than twice a second.")
        print(f"           Worst: {worst[0]:.0f}/s  \"{worst[2][:60]}\"")
        print("           On a persistent journal that is a sustained SD-card writer.")
        print("           Throttle it at the source, and consider Storage=volatile.")
    else:
        print("  VERDICT: nothing here is logging at a rate that would stress the card.")
        print("           If the board still went read-only, the writes came from")
        print("           somewhere other than the journal -- check for a process")
        print("           writing a file directly (bbot_watch's CSV, config saves).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
