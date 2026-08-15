#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
bbot_watch.py — 1 Hz system recorder for diagnosing lockups / SSH drops.

The problem this solves: when the board wedges, whatever explains it is in RAM
and dies with it. Journald on most BeagleBone images is volatile (no
/var/log/journal), so a reboot erases the evidence.

So: sample once a second, write a CSV line, and periodically fsync. When the
board dies, the last line on disk is close to the last moment it was alive —
and `--postmortem` finds the gap and shows you the run-up to it.

COST. This is a diagnostic tool, not a background service. fsync on an SD card
is expensive: at one fsync per second this used 10% of a BeagleBone core
sustained (19% peaks, 95 minutes of CPU over a 15 hour uptime) and drove system
time to 65%, which starved everything else on the box — ssh went sluggish and
the telemetry dashboard became unusable while the robot itself was fine.

So fsync now defaults to every 10 seconds, which bounds the loss to 10 samples
if the board dies. Use --paranoid for the old per-line behaviour when you are
actively hunting a lockup, and turn it off again afterwards.

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
    "net_iface", "net_carrier", "ipv4", "rx_bytes", "tx_bytes",
    "sshd_pid", "estab_ssh", "ctxt_per_s", "procs_running",
    "bot_pid", "boot_id",
]

# Changes exactly once per boot. The only unambiguous reboot detector — uptime
# can look like a reboot if the clock steps, and a hung board that recovers
# does not reboot at all.


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


def default_route_iface():
    """Interface holding the default route — the one SSH actually arrives on.

    Picking "first with carrier" is wrong on a BeagleBone: usb0 (the USB gadget)
    usually has carrier and sorts before the wifi/ethernet you connect over, so
    you would log the wrong link going down. /proc/net/route, no subprocess.
    """
    try:
        with open("/proc/net/route") as f:
            for line in f.readlines()[1:]:
                p = line.split()
                if len(p) > 2 and p[1] == "00000000":   # destination 0.0.0.0
                    return p[0]
    except Exception:
        pass
    return None


def net_state():
    """Reports the default-route interface, else the first one with carrier."""
    def stats(d):
        return (d,
                read(f"/sys/class/net/{d}/carrier", "?"),
                read(f"/sys/class/net/{d}/statistics/rx_bytes", ""),
                read(f"/sys/class/net/{d}/statistics/tx_bytes", ""))

    d = default_route_iface()
    if d and os.path.exists(f"/sys/class/net/{d}"):
        return stats(d)
    # No default route is itself a symptom — fall back so the drop stays visible.
    ifaces = [x for x in sorted(os.listdir("/sys/class/net")) if x != "lo"]
    for x in ifaces:
        if read(f"/sys/class/net/{x}/carrier", "?") == "1":
            return stats(x)
    return stats(ifaces[0]) if ifaces else ("", "", "", "")


def ipv4_of(iface):
    """Address without shelling out to `ip` — a DHCP change explains a timeout
    just as well as a hang, and looks identical from the Mac."""
    import socket, fcntl, struct
    try:
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            return socket.inet_ntoa(fcntl.ioctl(
                sk.fileno(), 0x8915, struct.pack("256s", iface[:15].encode()))[20:24])
        finally:
            sk.close()
    except Exception:
        return ""


def ssh_state():
    # PID not a bool: if sshd restarts between samples the PID changes, which
    # is the difference between "sshd died" and "sshd was never up".
    up = find_proc("sshd") or 0
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


def run(fsync_every=10):
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
    #
    # A boot marker goes in first so you can find where one boot ends and the
    # next begins; the file is append-only across reboots and is otherwise an
    # undifferentiated wall of lines.
    dmf = open(DMESG, "a", buffering=1)
    dmf.write(f"\n===== BOOT {read('/proc/sys/kernel/random/boot_id')[:8]} "
              f"at {datetime.datetime.now().isoformat(timespec='seconds')} =====\n")
    dmf.flush()
    os.fsync(dmf.fileno())

    dm = None
    for argv in (["stdbuf", "-oL", "dmesg", "--follow", "--time-format", "iso"],
                 ["dmesg", "--follow", "--time-format", "iso"],
                 ["dmesg", "--follow"]):
        # stdbuf -oL is the important part. dmesg writing to a pipe/file uses
        # full 4K buffering, so on a hard hang the last few KB — precisely the
        # lines that explain the hang — never reach the disk. Forcing line
        # buffering on the child costs nothing and makes the tail trustworthy.
        try:
            dm = subprocess.Popen(argv, stdout=dmf, stderr=subprocess.DEVNULL)
            break
        except Exception:
            continue
    if dm is None:
        print("bbot_watch: dmesg follow unavailable", file=sys.stderr)

    botcpu, stat = BotCpu(), Stat()
    print(f"bbot_watch: logging to {CSV} "
          f"(fsync every {fsync_every} sample{'s' if fsync_every != 1 else ''})",
          flush=True)
    since_sync = 0
    try:
        while True:
            t0 = time.monotonic()
            avail, swap_used = meminfo()
            la = read("/proc/loadavg").split()
            pct, rss, bstate = botcpu.sample()
            iface, carrier, rx, tx = net_state()
            sshd, estab = ssh_state()
            boot_id = read("/proc/sys/kernel/random/boot_id")[:8]
            ctxt, procs_r = stat.sample()
            row = [
                datetime.datetime.now().isoformat(timespec="seconds"),
                read("/proc/uptime").split(" ")[0],
                la[0] if len(la) > 0 else "", la[1] if len(la) > 1 else "",
                avail, swap_used,
                cpu_temp(), cpu_freq(),
                pct, rss, bstate,
                iface, carrier, ipv4_of(iface), rx, tx,
                sshd, estab, ctxt, procs_r,
                botcpu.pid or 0, boot_id,
            ]
            csv.write(",".join(str(x) for x in row) + "\n")
            csv.flush()
            since_sync += 1
            if since_sync >= fsync_every:
                os.fsync(csv.fileno())      # bounded loss, bounded cost
                since_sync = 0
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
        # boot_id changing is definitive. uptime can also step if the clock
        # is corrected, so it is only a fallback for older logs.
        if "boot_id" in idx and rows[i][idx["boot_id"]] and rows[i - 1][idx["boot_id"]]:
            rebooted = rows[i][idx["boot_id"]] != rows[i - 1][idx["boot_id"]]
        else:
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
    show = [c for c in ["ts", "load1", "mem_avail_kb", "temp_c",
                        "bot_cpu_pct", "bot_state", "bot_pid",
                        "net_carrier", "ipv4", "sshd_pid", "estab_ssh",
                        "ctxt_per_s", "procs_running"] if c in idx]
    for n, (i, dt, rebooted) in enumerate(gaps, 1):
        print(f"── event {n}: {'REBOOT' if rebooted else 'GAP'} of {dt:.0f}s "
              f"at {ts(rows[i-1])} → {ts(rows[i])}")
        w = [max(len(c), 12) for c in show]
        print("   " + " ".join(c.rjust(x) for c, x in zip(show, w)))
        for r in rows[max(0, i - 10):i]:
            print("   " + " ".join(str(r[idx[c]]).rjust(x) for c, x in zip(show, w)))
        print()
    # Turn the evidence into the one distinction that matters from the Mac:
    # "connection refused" and "connection timed out" have different causes.
    last = rows[gaps[-1][0] - 1]
    prev = rows[max(0, gaps[-1][0] - 6)]
    def g(r, k):
        return r[idx[k]] if k in idx else ""
    print("Reading of the last event:")
    if gaps[-1][2]:
        print("  Board REBOOTED (boot_id changed).")
        print("  -> From the Mac this looks like TIMEOUT while it is down, then")
        print("     REFUSED for the few seconds after the network is up but")
        print("     before sshd starts. Getting both intermittently is expected.")
    else:
        print("  Board did NOT reboot — it stopped logging and came back.")
        if g(last, "net_carrier") == "0":
            print("  -> net_carrier was already 0: the LINK dropped, board alive. TIMEOUT.")
        elif g(last, "ipv4") and g(prev, "ipv4") and g(last, "ipv4") != g(prev, "ipv4"):
            print(f"  -> IP changed {g(prev,'ipv4')} -> {g(last,'ipv4')}. You were talking")
            print("     to a stale address. TIMEOUT, and mDNS may still resolve the old one.")
        elif g(last, "sshd_pid") in ("0", ""):
            print("  -> sshd was GONE while the kernel kept logging. That is REFUSED.")
        else:
            print("  -> kernel stopped scheduling us but did not reboot: hard hang or")
            print("     power sag that recovered. TIMEOUT. Check dmesg below.")
    print()
    # The kernel's own last words, if we captured them.
    try:
        died_at = ts(rows[gaps[-1][0] - 1])
        keep = []
        with open(DMESG) as f:
            for line in f:
                line = line.rstrip("\n")
                if line.startswith("====="):
                    keep.append(line)
                    continue
                stamp = line.split(" ", 1)[0].split(",")[0].split("+")[0]
                try:
                    if datetime.datetime.fromisoformat(stamp) <= died_at:
                        keep.append(line)
                except ValueError:
                    keep.append(line)
        if keep:
            print(f"Last {min(15, len(keep))} kernel messages before the event:")
            for line in keep[-15:]:
                print("   " + line[:150])
            print()
    except FileNotFoundError:
        pass
    print(f"Full kernel log: {DMESG}")
    print("Also check:  journalctl -b -1 -e     (needs persistent journal)")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--postmortem", action="store_true",
                    help="analyse an existing log instead of recording")
    ap.add_argument("--gap", type=float, default=5.0,
                    help="seconds of silence that counts as a death (default 5)")
    ap.add_argument("--fsync-every", type=int, default=10, metavar="N",
                    help="fsync every N samples (default 10). Lower is safer but "
                         "costs real CPU on an SD card")
    ap.add_argument("--paranoid", action="store_true",
                    help="fsync every line. Loses nothing on a hard lockup, but "
                         "costs ~10%% of a BeagleBone core -- use while hunting a "
                         "lockup, not permanently")
    a = ap.parse_args()
    every = 1 if a.paranoid else max(1, a.fsync_every)
    sys.exit(postmortem(a.gap) if a.postmortem else (run(every) or 0))
