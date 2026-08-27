#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
rf_check.py — which link metric actually moves when the RC transmitter is on?

    sudo ./tools/rf_check.py            30s off, 30s on
    sudo ./tools/rf_check.py 20         20s in each half

Samples every candidate metric at 1 Hz, tells you when to flip the
transmitter, and finishes by reporting which metrics changed between the two
halves — and by how much.

Why this exists, twice over:

  1. With the transmitter on, this board's ssh echo latency runs to seconds.
     Anything that needs you to type in the failure state is unusable, so the
     tool announces the transition and you only read the result afterward.

  2. The first attempt at a live dashboard readout chose its metrics from
     /proc/net/wireless, which on this board's rtw88 dongle reports a permanent
     0 for `retry` and -256 (the "no data" sentinel) for the noise floor. The
     result was a widget reporting a healthy link while the link crawled. Pick
     the metric by measuring which one responds; never assume the textbook
     counter is populated by the driver in front of you.

The known-good signature of 2.4 GHz interference is NOT a weaker signal — an
interferer raises the noise floor rather than attenuating you. It is bitrate
collapse with retry/failure counts climbing while `signal` barely moves. That
is why bitrate is sampled here at all, and why the summary says so explicitly.
"""
import os, re, subprocess, sys, time

HALF = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


def which_iw():
    """iw lives in /usr/sbin, which is not on a normal user's PATH.

    Worth spelling out: `iw: command not found` from a user shell means
    "not on your PATH", not "not installed". The same trap applies to
    swapon, ip and ethtool on this image.
    """
    for p in ("/usr/sbin/iw", "/sbin/iw", "/usr/bin/iw", "/bin/iw"):
        if os.path.exists(p):
            return p
    from shutil import which
    return which("iw")


IW = which_iw()


def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=10).stdout
    except Exception:
        return ""


def default_iface():
    try:
        for line in open("/proc/net/route"):
            f = line.split()
            if len(f) > 1 and f[1] == "00000000":
                return f[0]
    except OSError:
        pass
    return ""


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def sample(iface):
    """One observation. Gauges absolute; names starting c_ are raw counters."""
    o = {}

    if IW:
        txt = (run([IW, "dev", iface, "link"]) +
               run([IW, "dev", iface, "station", "dump"]))
        pats = {
            "signal_dbm":    r"signal:\s*(-?\d+)",
            "tx_mbps":       r"tx bitrate:\s*([\d.]+)",
            "rx_mbps":       r"rx bitrate:\s*([\d.]+)",
            "expected_mbps": r"expected throughput:\s*([\d.]+)",
            "beacon_loss":   r"beacon loss:\s*(\d+)",
            "c_tx_retries":  r"tx retries:\s*(\d+)",
            "c_tx_failed":   r"tx failed:\s*(\d+)",
        }
        for k, pat in pats.items():
            m = re.search(pat, txt)
            if m:
                o[k] = num(m.group(1))

    # /proc/net/wireless is kept here specifically so the run PROVES whether it
    # moves, rather than us assuming either way a second time.
    for line in read("/proc/net/wireless").splitlines():
        if ":" not in line:
            continue
        name, _, rest = line.strip().partition(":")
        if name.strip() != iface:
            continue
        f = rest.split()
        if len(f) >= 9:
            o["wext_q"] = num(f[1].rstrip("."))
            o["wext_level"] = num(f[2].rstrip("."))
            o["c_wext_retry"] = num(f[7])
            o["c_wext_misc"] = num(f[8])

    st = f"/sys/class/net/{iface}/statistics/"
    for k in ("tx_errors", "tx_dropped", "rx_dropped", "rx_errors",
              "tx_packets", "rx_packets", "tx_bytes", "rx_bytes"):
        v = num(read(st + k).strip())
        if v is not None:
            o["c_" + k] = v
    return o


def collect(iface, seconds, label):
    """{metric: [values]} — counters converted to per-second deltas."""
    rows = []
    t_end = time.monotonic() + seconds
    prev = sample(iface)
    prev_t = time.monotonic()
    while time.monotonic() < t_end:
        time.sleep(1.0)
        cur = sample(iface)
        now = time.monotonic()
        dt = now - prev_t
        r = {}
        for k, v in cur.items():
            if k.startswith("c_"):
                p = prev.get(k)
                r[k[2:] + "/s"] = max(0.0, (v - p) / dt) if p is not None else 0.0
            else:
                r[k] = v
        rows.append(r)
        prev, prev_t = cur, now
        sig, tx = r.get("signal_dbm"), r.get("tx_mbps")
        print(f"    {label}  sig {('--' if sig is None else f'{sig:.0f}'):>4}  "
              f"tx {('--' if tx is None else f'{tx:.1f}'):>6} Mbps  "
              f"retries {r.get('tx_retries/s', 0):7.0f}/s  "
              f"failed {r.get('tx_failed/s', 0):5.0f}/s", flush=True)
    agg = {}
    for r in rows:
        for k, v in r.items():
            if v is not None:
                agg.setdefault(k, []).append(v)
    return agg


def mean(xs):
    return sum(xs) / len(xs) if xs else None


def main():
    iface = default_iface()
    if not iface:
        print("  no default route — nothing to measure")
        return 1
    print(f"\n  interface: {iface}    "
          f"iw: {IW or 'NOT FOUND (sudo apt install iw)'}")
    if os.geteuid() != 0:
        print("  note: not root — `iw station dump` often returns nothing "
              "without it. Use sudo.")

    input(f"\n  Transmitter OFF, then press Enter (samples {HALF:.0f}s) ... ")
    off = collect(iface, HALF, "off")

    print(f"\n  >>> TURN THE TRANSMITTER ON NOW — sampling {HALF:.0f}s <<<\n",
          flush=True)
    on = collect(iface, HALF, "ON ")

    print()
    print(f"  {'metric':<20}{'TX off':>12}{'TX on':>12}{'change':>12}")
    print("  " + "-" * 58)
    moved = []
    for k in sorted(set(off) | set(on)):
        a, b = mean(off.get(k, [])), mean(on.get(k, []))
        if a is None or b is None:
            continue
        # "Did it move" must be scale-free: retries going 0 -> 40 and bitrate
        # going 65 -> 6 are both decisive, and any single absolute threshold
        # would miss one of them.
        base = max(abs(a), abs(b), 1e-9)
        frac = abs(b - a) / base
        flag = ""
        if frac > 0.25 and abs(b - a) > 0.5:
            flag = "   <-- MOVED"
            moved.append((frac, k, a, b))
        print(f"  {k:<20}{a:>12.1f}{b:>12.1f}{b - a:>+12.1f}{flag}")

    print()
    if not moved:
        print("  NOTHING MOVED. The transmitter is not measurably touching this")
        print("  link in anything the driver exposes. If the shell is still slow")
        print("  with it on, then the latency is not coming from the radio, and")
        print("  no amount of band separation will fix it — look elsewhere.")
    else:
        moved.sort(reverse=True)
        print("  Responded, biggest relative change first:")
        for frac, k, a, b in moved:
            print(f"    {k:<18} {a:.1f} -> {b:.1f}   ({frac * 100:.0f}%)")
        print()
        sig_a, sig_b = mean(off.get("signal_dbm", [])), mean(on.get("signal_dbm", []))
        sig_moved = (sig_a is not None and sig_b is not None
                     and abs(sig_b - sig_a) > 6)
        rate_moved = any(k in ("tx_mbps", "rx_mbps", "expected_mbps")
                         for _, k, _, _ in moved)
        if rate_moved and not sig_moved:
            print("  That is the interference signature: bitrate collapsed while")
            print("  signal held. An interferer lifts the noise floor rather than")
            print("  attenuating you, so the rate control backs off while the")
            print("  received signal strength looks fine. Fix is band separation")
            print("  (move the AP and dongle to 5 GHz) or physical distance")
            print("  between the dongle and the RC antenna — not software.")
        elif sig_moved:
            print("  signal_dbm itself moved substantially, which is range or")
            print("  antenna rather than interference. Check the dongle's")
            print("  placement and whether the TX is physically shadowing it.")
        else:
            print("  Something responded but not the bitrate and not the signal.")
            print("  Report this table — the useful metric on this driver is")
            print("  whatever is at the top of the list above, and the dashboard")
            print("  readout should be rebuilt around that rather than around")
            print("  the counters /proc/net/wireless happens to offer.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print()
