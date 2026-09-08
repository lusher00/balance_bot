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
disk_load.py — what is actually writing to the SD card, and which process?

Run on the bot. Measures bytes reaching the block device, and attributes them to
processes, over a sample window. Run it with the transmitter off and again with
it on to see what changes.

    sudo ./tools/disk_load.py            # 15 s sample
    sudo ./tools/disk_load.py 30

Why measure instead of reason: on this board a read-only remount has been chased
through three plausible-but-wrong culprits (a journald flood, config saves, the
display's log file). Each was a real mechanism that could have done it, and none
of them was doing it. /proc/diskstats and /proc/<pid>/io settle it -- they say
what reached the device and who asked, with no theory in between.

WHAT THE NUMBERS MEAN
  device writes    sectors*512 from /proc/diskstats, i.e. what the card is
                   really being asked to store. Includes filesystem metadata and
                   journal commits, which per-process counters miss.
  write_bytes      per-process, from /proc/<pid>/io: bytes this process caused
                   to be sent to storage. Does not include its share of
                   filesystem overhead, so the per-process total is normally
                   somewhat LESS than the device total. A large gap means
                   metadata/journal churn -- many small writes rather than few
                   big ones, which is the pattern that wears a card fastest.
  cancelled        write_bytes that never reached the device because the file
                   was truncated or deleted first. High values mean a file being
                   rewritten repeatedly.
"""
import os, sys, time

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0


def diskstats():
    """device -> sectors written. Only whole devices, not partitions."""
    out = {}
    try:
        with open("/proc/diskstats") as f:
            for line in f:
                p = line.split()
                if len(p) < 10:
                    continue
                name = p[2]
                if not name.startswith(("mmcblk", "sd", "vd", "nvme", "hd")):
                    continue
                # Count whole devices, not partitions, or the same write is
                # counted twice. Partition naming differs by device class:
                # mmcblk0p1 / nvme0n1p1 use "p<n>"; sda1 / vda1 just append a
                # digit -- but mmcblk0 and nvme0n1 END in a digit themselves.
                if name.startswith(("mmcblk", "nvme")):
                    import re as _re
                    if _re.search(r"p\d+$", name):
                        continue
                elif name[-1].isdigit():
                    continue
                out[name] = int(p[9])    # field 10: sectors written
    except OSError:
        pass
    return out


def proc_io():
    """pid -> (comm, write_bytes, cancelled_write_bytes)"""
    out = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/comm") as f:
                comm = f.read().strip()
            wb = cw = 0
            with open(f"/proc/{d}/io") as f:
                for line in f:
                    if line.startswith("write_bytes:"):
                        wb = int(line.split()[1])
                    elif line.startswith("cancelled_write_bytes:"):
                        cw = int(line.split()[1])
            out[int(d)] = (comm, wb, cw)
        except Exception:
            continue          # process exited, or not permitted
    return out


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if abs(n) < 1024:
            return f"{n:.0f}{unit}" if unit == "B" else f"{n:.1f}{unit}"
        n /= 1024.0
    return f"{n:.1f}TB"


def main():
    if os.geteuid() != 0:
        print("  note: not root — /proc/<pid>/io is unreadable for other users'")
        print("        processes, so attribution will be incomplete. Use sudo.\n")

    # Where do the write-heavy paths actually live? If /tmp and /var/log are on
    # the root filesystem, everything written there hits the card.
    print("  mount points that matter:")
    try:
        with open("/proc/mounts") as f:
            mounts = [l.split() for l in f]
        for want in ("/", "/tmp", "/var/log", "/run"):
            hit = [m for m in mounts if m[1] == want]
            if hit:
                dev, mp, fstype = hit[-1][0], hit[-1][1], hit[-1][2]
                oncard = fstype not in ("tmpfs", "ramfs", "overlay", "devtmpfs")
                print(f"    {mp:<10}{dev:<22}{fstype:<10}"
                      f"{'ON THE CARD' if oncard else 'RAM — free to write'}")
            else:
                print(f"    {want:<10}(not a separate mount — part of /)")
    except OSError:
        pass

    print()
    print(f"  sampling {SECS:.0f}s ...", flush=True)
    d0, p0 = diskstats(), proc_io()
    time.sleep(SECS)
    d1, p1 = diskstats(), proc_io()

    print()
    dev_total = 0
    for dev in sorted(d1):
        sectors = d1[dev] - d0.get(dev, 0)
        b = sectors * 512
        dev_total += b
        print(f"  device {dev:<10}{human(b):>10} written  "
              f"({human(b / SECS)}/s)")

    rows = []
    for pid, (comm, wb, cw) in p1.items():
        if pid not in p0:
            continue
        d_wb = wb - p0[pid][1]
        d_cw = cw - p0[pid][2]
        if d_wb > 0 or d_cw > 0:
            rows.append((d_wb, d_cw, pid, comm))
    rows.sort(reverse=True)

    print()
    if rows:
        print(f"  {'process':<20}{'pid':>7}{'written':>12}{'per sec':>12}{'cancelled':>12}")
        print("  " + "-" * 63)
        for d_wb, d_cw, pid, comm in rows[:12]:
            print(f"  {comm[:19]:<20}{pid:>7}{human(d_wb):>12}"
                  f"{human(d_wb / SECS):>12}{human(d_cw):>12}")
        proc_total = sum(r[0] for r in rows)
    else:
        print("  no process reported any write. Either nothing is writing, or")
        print("  you are not root and cannot see their counters.")
        proc_total = 0

    # ── verdict ─────────────────────────────────────────────────────────────
    print()
    rate = dev_total / SECS
    if dev_total == 0:
        print("  VERDICT: nothing reached the card during this sample. If the board")
        print("           still misbehaves, it is not disk write volume.")
    elif rate < 4 * 1024:
        print(f"  VERDICT: {human(rate)}/s to the card — negligible. Whatever is wrong,")
        print("           it is not write volume.")
    else:
        print(f"  VERDICT: {human(rate)}/s sustained to the card.")
        if rows:
            top = rows[0]
            share = 100.0 * top[0] / dev_total if dev_total else 0
            print(f"           Biggest writer: {top[3]} (pid {top[2]}), "
                  f"{human(top[0] / SECS)}/s, ~{share:.0f}% of it.")
        gap = dev_total - proc_total
        if proc_total and gap > proc_total:
            print(f"           {human(gap)} of it is NOT attributable to any process —")
            print("           that is filesystem metadata and journal commits, i.e. many")
            print("           small scattered writes. That pattern stalls an SD card far")
            print("           worse than the byte count suggests.")
    print()
    print("  Run again with the transmitter in the other state and compare.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
