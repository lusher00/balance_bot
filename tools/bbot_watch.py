#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
bbot_watch.py — 1 Hz system recorder for diagnosing lockups / SSH drops.

The problem this solves: when the board wedges, whatever explains it is in RAM
and dies with it. Journald on most BeagleBone images is volatile (no
/var/log/journal), so a reboot erases the evidence.

So: sample once a second, write a CSV line, and fsync it. Every line is on
disk before the next one is taken. When the board dies, the last line written
is the last moment it was alive — and `--postmortem` finds the gap and shows
you the run-up to it.

  run:        sudo ./bbot_watch.py
  after boot: ./bbot_watch.py --postmortem

Columns are cheap to read and cheap to collect; nothing here opens a socket or
shells out on the hot path, so the recorder itself cannot be what hangs you.
"""

import os, sys, time, glob, subprocess, argparse, datetime

LOG_DIR  = "/var/log/bbot_watch"
CSV      = os.path.join(LOG_DIR, "system.csv")
DMESG    = os.path.join(LOG_DIR, "dmesg.log")
MAX_MB   = 32

FIELDS = [
    "ts", "uptime_s", "load1", "load5",
    "mem_avail_kb", "swap_used_kb",
    "temp_c", "cpu_freq_khz",
    "bot_cpu_pct", "bot_rss_kb", "bot_state",
    "net_iface", "net_carrier", "rx_bytes", "tx_bytes",
    "sshd_up", "estab_ssh", "ctxt_per_s", "procs_running",
]


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:
        return default


def first_glob(pattern):
    m = sorted(glob.glob(pattern))
    return m[0] if m else None


def cpu_temp():
    # BeagleBone exposes this inconsistently across images; try the usual spots.
    for p in ("/sys/class/thermal/thermal_zone0/temp",
              "/sys/devices/virtual/thermal/thermal_zone0/temp"):
        v = read(p)
        if v.isdigit():
            t = int(v)
            return round(t / 1000.0, 1) if t > 1000 else float(t)
    return ""


def cpu_freq():
    return read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "")


def meminfo():
    avail = swap_total = swap_free = 0
    for line in read("/proc/meminfo").splitlines():
        k, _, v = line.partition(":")
        v = v.strip().split(" ")[0]
        if k == "MemAvailable":
            avail = int(v)
        elif k == "SwapTotal":
            swap_total = int(v)
        elif k == "SwapFree":
            swap_free = int(v)
    return avail, swap_total - swap_free


def find_bot():
    """PID of balance_bot, or None. Reads /proc directly — no pgrep subprocess."""
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/comm") as f:
                if f.read().strip() == "balance_bot":
                    return int(d)
        except Exception:
            continue
    return None


class BotCpu:
    """Delta-based CPU% for balance_bot. Absolute jiffies are useless alone."""
    def __init__(self):
        self.pid = None
        self.prev_proc = None
        self.prev_total = None
        self.hz = os.sysconf("SC_CLK_TCK")

    def sample(self):
        pid = find_bot()
        if pid != self.pid:
            self.pid, self.prev_proc, self.prev_total = pid, None, None
        if pid is None:
            return "", "", "down"
        try:
            stat = read(f"/proc/{pid}/stat").split()
            utime, stime, state = int(stat[13]), int(stat[14]), stat[2]
            rss_pages = int(stat[23])
            proc_j = utime + stime
            total_j = time.monotonic() * self.hz
            pct = ""
            if self.prev_proc is not None:
                dp, dt = proc_j - self.prev_proc, total_j - self.prev_total
                if dt > 0:
                    pct = round(100.0 * dp / dt, 1)
            self.prev_proc, self.prev_total = proc_j, total_j
            return pct, rss_pages * (os.sysconf("SC_PAGE_SIZE") // 1024), state
        except Exception:
            return "", "", "?"


def net_state():
    """First non-loopback iface that has an operstate. Reports carrier + counters."""
    for d in sorted(os.listdir("/sys/class/net")):
        if d == "lo":
            continue
        carrier = read(f"/sys/class/net/{d}/carrier", "?")
        rx = read(f"/sys/class/net/{d}/statistics/rx_bytes", "")
        tx = read(f"/sys/class/net/{d}/statistics/tx_bytes", "")
        if carrier == "1":
            return d, carrier, rx, tx
    # nothing has carrier — still report the first iface so the drop is visible
    ifaces = [d for d in sorted(os.listdir("/sys/class/net")) if d != "lo"]
    if ifaces:
        d = ifaces[0]
        return (d, read(f"/sys/class/net/{d}/carrier", "0"),
                read(f"/sys/class/net/{d}/statistics/rx_bytes", ""),
                read(f"/sys/class/net/{d}/statistics/tx_bytes", ""))
    return "", "", "", ""


def ssh_state():
    up = 1 if find_proc("sshd") else 0
    n = 0
    try:
        # count ESTABLISHED (state 01) sockets on port 22 (0x16)
        with open("/proc/net/tcp") as f:
            for line in f.readlines()[1:]:
                p = line.split()
                if p[3] == "01" and p[1].split(":")[1] == "0016":
                    n += 1
    except Exception:
        pass
    return up, n


def find_proc(name):
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/comm") as f:
                if f.read().strip() == name:
                    return int(d)
        except Exception:
            continue
    return None


class Stat:
    """Context switches per second and runnable process count."""
    def __init__(self):
        self.prev_ctxt = None
        self.prev_t = None

    def sample(self):
        ctxt = procs_r = ""
        for line in read("/proc/stat").splitlines():
            if line.startswith("ctxt "):
                ctxt = int(line.split()[1])
            elif line.startswith("procs_running "):
                procs_r = int(line.split()[1])
        rate = ""
        now = time.monotonic()
        if self.prev_ctxt is not None and isinstance(ctxt, int):
            dt = now - self.prev_t
            if dt > 0:
                rate = int((ctxt - self.prev_ctxt) / dt)
        if isinstance(ctxt, int):
            self.prev_ctxt, self.prev_t = ctxt, now
        return rate, procs_r


def rotate(path):
    try:
        if os.path.getsize(path) > MAX_MB * 1024 * 1024:
            os.replace(path, path + ".1")
    except FileNotFoundError:
        pass


def run():
    os.makedirs(LOG_DIR, exist_ok=True)
    rotate(CSV)
    new = not os.path.exists(CSV) or os.path.getsize(CSV) == 0
    csv = open(CSV, "a", buffering=1)
    if new:
        csv.write(",".join(FIELDS) + "\n")
        csv.flush()
        os.fsync(csv.fileno())

    # Kernel ring buffer follower — brownouts, USB/MMC resets, OOM kills and
    # panics land here and nowhere else.
    dm = None
    try:
        dm = subprocess.Popen(["dmesg", "--follow", "--time-format", "iso"],
                              stdout=open(DMESG, "a", buffering=1),
                              stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"bbot_watch: dmesg follow unavailable ({e})", file=sys.stderr)

    botcpu, stat = BotCpu(), Stat()
    print(f"bbot_watch: logging to {CSV} (fsync per line)", flush=True)
    try:
        while True:
            t0 = time.monotonic()
            avail, swap_used = meminfo()
            la = read("/proc/loadavg").split()
            pct, rss, bstate = botcpu.sample()
            iface, carrier, rx, tx = net_state()
            sshd, estab = ssh_state()
            ctxt, procs_r = stat.sample()
            row = [
                datetime.datetime.now().isoformat(timespec="seconds"),
                read("/proc/uptime").split(" ")[0],
                la[0] if len(la) > 0 else "", la[1] if len(la) > 1 else "",
                avail, swap_used,
                cpu_temp(), cpu_freq(),
                pct, rss, bstate,
                iface, carrier, rx, tx,
                sshd, estab, ctxt, procs_r,
            ]
            csv.write(",".join(str(x) for x in row) + "\n")
            csv.flush()
            os.fsync(csv.fileno())          # the whole point — survive the hang
            time.sleep(max(0.0, 1.0 - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if dm:
            dm.terminate()
        csv.close()


def postmortem(gap_s=5.0):
    if not os.path.exists(CSV):
        print(f"no log at {CSV} — was bbot_watch running?")
        return 1
    rows = [l.rstrip("\n").split(",") for l in open(CSV)]
    hdr, rows = rows[0], [r for r in rows[1:] if len(r) == len(rows[0])]
    if not rows:
        print("log is empty")
        return 1
    idx = {k: i for i, k in enumerate(hdr)}

    def ts(r):
        return datetime.datetime.fromisoformat(r[idx["ts"]])

    # A gap in wall-clock, or uptime going backwards (= reboot), marks a death.
    gaps = []
    for i in range(1, len(rows)):
        dt = (ts(rows[i]) - ts(rows[i - 1])).total_seconds()
        try:
            rebooted = float(rows[i][idx["uptime_s"]]) < float(rows[i - 1][idx["uptime_s"]])
        except ValueError:
            rebooted = False
        if dt > gap_s or rebooted:
            gaps.append((i, dt, rebooted))

    if not gaps:
        print(f"No gaps > {gap_s}s and no reboots in {len(rows)} samples "
              f"({ts(rows[0])} -> {ts(rows[-1])}). Board has not died since logging began.")
        return 0

    print(f"{len(gaps)} event(s) found in {len(rows)} samples\n")
    show = ["ts", "load1", "mem_avail_kb", "temp_c", "cpu_freq_khz",
            "bot_cpu_pct", "bot_state", "net_carrier", "estab_ssh",
            "ctxt_per_s", "procs_running"]
    for n, (i, dt, rebooted) in enumerate(gaps, 1):
        print(f"── event {n}: {'REBOOT' if rebooted else 'GAP'} of {dt:.0f}s "
              f"at {ts(rows[i-1])} → {ts(rows[i])}")
        w = [max(len(c), 12) for c in show]
        print("   " + " ".join(c.rjust(x) for c, x in zip(show, w)))
        for r in rows[max(0, i - 10):i]:
            print("   " + " ".join(str(r[idx[c]]).rjust(x) for c, x in zip(show, w)))
        print()
    print(f"Kernel messages around those times: {DMESG}")
    print("Also check:  journalctl -b -1 -e     (needs persistent journal)")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--postmortem", action="store_true",
                    help="analyse an existing log instead of recording")
    ap.add_argument("--gap", type=float, default=5.0,
                    help="seconds of silence that counts as a death (default 5)")
    a = ap.parse_args()
    sys.exit(postmortem(a.gap) if a.postmortem else (run() or 0))
