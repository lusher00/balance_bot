#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
sbus_load.py — what does turning the transmitter on actually cost this board?

Run it on the bot. It samples with the transmitter OFF, then ON, and prints the
difference. One command, no reading numbers off a screen while the box is too
busy to echo your keystrokes.

    sudo ./tools/sbus_load.py

Why this exists: "the whole board grinds when the TX is on" has a small number
of distinguishable causes, and CPU% tells them apart badly. This board has
already been bitten once by a fault that showed 60%+ IDLE CPU while ssh was
unusable -- the SBUS driver was doing one read() syscall per byte, and the only
number that showed it was the context-switch rate.

So this measures the things that actually separate the causes:

  interrupts/s   a UART interrupting per byte instead of per FIFO-full
  ctxt/s         a thread waking per byte instead of per block
  serial errors  framing/parity/overrun -- wrong baud, or a marginal inverter,
                 which also makes the kernel log, which writes to the SD card
  cpu / iowait   distinguishes compute-bound from blocked-on-disk
  loop_hz        whether the control loop itself is being starved

Each cause implies a different fix, and three of the five are invisible in top.
"""
import os, re, sys, time, json, socket

SAMPLE_S = float(os.environ.get("SAMPLE_S", 10))


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return default


def proc_stat():
    out = {"cpu": None, "ctxt": 0, "intr": 0, "procs_running": 0}
    for line in read("/proc/stat").splitlines():
        if line.startswith("cpu "):
            v = [int(x) for x in line.split()[1:]]
            total = sum(v)
            idle = v[3] + (v[4] if len(v) > 4 else 0)
            iowait = v[4] if len(v) > 4 else 0
            out["cpu"] = (total, idle, iowait)
        elif line.startswith("ctxt "):
            out["ctxt"] = int(line.split()[1])
        elif line.startswith("intr "):
            out["intr"] = int(line.split()[1])
        elif line.startswith("procs_running "):
            out["procs_running"] = int(line.split()[1])
    return out


def interrupts():
    """Per-IRQ counts, summed across CPUs, keyed by 'irq label'."""
    d = {}
    for line in read("/proc/interrupts").splitlines()[1:]:
        parts = line.split()
        if not parts or not parts[0].rstrip(":").isdigit():
            continue
        irq = parts[0].rstrip(":")
        nums, label = [], []
        for p in parts[1:]:
            if p.isdigit():
                nums.append(int(p))
            else:
                label.append(p)
        d[f"{irq} {' '.join(label)}"] = sum(nums)
    return d


def serial_errors():
    """Per-port error counters. Root only; absent on some kernels."""
    d = {}
    for line in read("/proc/tty/driver/serial").splitlines():
        m = re.match(r"\s*(\d+):\s*uart:(\S+)", line)
        if not m:
            continue
        port = f"ttyS{m.group(1)}"
        errs = {}
        for k in ("fe", "pe", "oe", "brk"):
            mm = re.search(rf"\b{k}:(\d+)", line)
            if mm:
                errs[k] = int(mm.group(1))
        if errs:
            d[port] = errs
    return d


def loop_hz(timeout=3.0):
    """One telemetry packet off the unix socket, for loop_hz. Optional."""
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect("/tmp/balance_bot.sock")
        buf = b""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            buf += s.recv(65536)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    o = json.loads(line)
                except Exception:
                    continue
                if o.get("type") == "telemetry":
                    s.close()
                    return (o.get("system") or {}).get("loop_hz")
        s.close()
    except Exception:
        pass
    return None


def sample(label):
    print(f"  sampling {SAMPLE_S:.0f}s with the transmitter {label} ...", flush=True)
    a_stat, a_irq, a_ser = proc_stat(), interrupts(), serial_errors()
    t0 = time.monotonic()
    hz = loop_hz()
    time.sleep(max(0.0, SAMPLE_S - (time.monotonic() - t0)))
    dt = time.monotonic() - t0
    b_stat, b_irq, b_ser = proc_stat(), interrupts(), serial_errors()

    r = {"dt": dt, "loop_hz": hz,
         "ctxt": (b_stat["ctxt"] - a_stat["ctxt"]) / dt,
         "intr": (b_stat["intr"] - a_stat["intr"]) / dt,
         "procs_running": b_stat["procs_running"]}
    if a_stat["cpu"] and b_stat["cpu"]:
        dtot = b_stat["cpu"][0] - a_stat["cpu"][0]
        didle = b_stat["cpu"][1] - a_stat["cpu"][1]
        diow = b_stat["cpu"][2] - a_stat["cpu"][2]
        r["cpu"] = 100.0 * (dtot - didle) / dtot if dtot else 0.0
        r["iowait"] = 100.0 * diow / dtot if dtot else 0.0
    r["irq"] = {k: (b_irq[k] - a_irq.get(k, 0)) / dt
                for k in b_irq if b_irq[k] - a_irq.get(k, 0) > 0}
    r["ser"] = {p: {k: b_ser[p][k] - a_ser.get(p, {}).get(k, 0) for k in b_ser[p]}
                for p in b_ser}
    return r


def main():
    if os.geteuid() != 0:
        print("  note: not root — /proc/tty/driver/serial (framing/parity/overrun")
        print("        counters) will be unreadable. Re-run with sudo for those.\n")

    input("  Turn the transmitter OFF, then press Enter... ")
    off = sample("OFF")
    print()
    input("  Now turn the transmitter ON, then press Enter... ")
    on = sample("ON")

    def row(name, a, b, unit="", dp=0, warn=None):
        if a is None or b is None:
            return
        delta = b - a
        flag = ""
        if warn is not None and delta > warn:
            flag = "   <-- "
        print(f"  {name:<16}{a:>12.{dp}f}{b:>12.{dp}f}{delta:>+12.{dp}f} {unit}{flag}")

    print()
    print(f"  {'':<16}{'TX off':>12}{'TX on':>12}{'change':>12}")
    print("  " + "-" * 52)
    row("interrupts/s", off["intr"], on["intr"], warn=3000)
    row("ctxt/s", off["ctxt"], on["ctxt"], warn=3000)
    row("cpu %", off.get("cpu"), on.get("cpu"), dp=1)
    row("iowait %", off.get("iowait"), on.get("iowait"), dp=1)
    row("procs running", off["procs_running"], on["procs_running"])
    if off["loop_hz"] is not None and on["loop_hz"] is not None:
        row("loop_hz", off["loop_hz"], on["loop_hz"], dp=1)

    # Which IRQ moved? That names the device without needing to know its number.
    print()
    print("  biggest interrupt sources with the TX on:")
    movers = sorted(((on["irq"].get(k, 0) - off["irq"].get(k, 0), k)
                     for k in set(on["irq"]) | set(off["irq"])), reverse=True)
    for d, k in movers[:5]:
        if abs(d) < 1:
            continue
        print(f"    {k:<34}{on['irq'].get(k,0):>9.0f}/s   ({d:+.0f})")

    if on["ser"]:
        print()
        print("  serial error counters (delta over the TX-on sample):")
        for p, e in on["ser"].items():
            bad = sum(e.values())
            print(f"    {p:<10} framing={e.get('fe',0)} parity={e.get('pe',0)} "
                  f"overrun={e.get('oe',0)} break={e.get('brk',0)}"
                  f"{'   <-- signal or baud problem' if bad else ''}")

    # ── verdict ──────────────────────────────────────────────────────────────
    print()
    d_int = on["intr"] - off["intr"]
    d_ctx = on["ctxt"] - off["ctxt"]
    d_cpu = (on.get("cpu") or 0) - (off.get("cpu") or 0)
    ser_bad = sum(sum(e.values()) for e in on["ser"].values()) if on["ser"] else 0

    if ser_bad > 0:
        print("  VERDICT: the UART is reporting framing/parity/overrun errors while the")
        print("           transmitter is on. That is a signal or baud problem, not load.")
        print("           SBUS is 100000 baud 8E2; this driver runs the port at a custom")
        print("           divisor off B38400. If the divisor is off, every frame errors,")
        print("           the kernel logs it, and the logging is what grinds the board.")
    elif d_int > 5000:
        print(f"  VERDICT: interrupts jumped {d_int:.0f}/s. The UART is interrupting far more")
        print("           often than a 25-byte frame every 7 ms needs (~140/s). That is a")
        print("           per-byte interrupt -- the RX FIFO trigger level is not being used.")
    elif d_ctx > 3000:
        print(f"  VERDICT: context switches jumped {d_ctx:.0f}/s with little else moving.")
        print("           Something is waking per byte rather than per block. This is the")
        print("           same shape as the read()-per-byte bug, in a different place.")
    elif d_cpu > 30:
        print(f"  VERDICT: CPU rose {d_cpu:.0f} points. Ordinary load, not a latency fault --")
        print("           look at what is running: the telemetry rates, or node.")
    else:
        print("  VERDICT: the transmitter changed almost nothing here.")
    print()

    # Judge the ABSOLUTE level too, not just the change. An earlier version of
    # this stopped at "nothing moved much" -- true, and useless, when BOTH
    # samples were already in the range that had previously made this board
    # unusable. A comparison tool that never looks at where the baseline sits
    # will happily report that a permanently broken board is stable.
    worst_ctxt = max(off["ctxt"], on["ctxt"])
    worst_cpu = max(off.get("cpu") or 0, on.get("cpu") or 0)
    if worst_ctxt > 5000:
        print(f"  BUT THE BASELINE IS BAD: {worst_ctxt:.0f} context switches/s, with the")
        print("  transmitter in EITHER state. For reference, the SBUS read()-per-byte")
        print("  bug on this board measured 6983-7156/s and made ssh sluggish and the")
        print("  dashboard erratic while CPU sat mostly idle.")
        print(f"  CPU is {worst_cpu:.0f}% on a single core with the robot doing nothing.")
        print()
        print("  So the transmitter is not your problem -- the board is in this state")
        print("  all the time. Find the source:")
        print("      sudo ./tools/cpu_top.py 20             which process")
        print("      sudo ./tools/cpu_top.py 20 --threads   which thread inside it")
    elif worst_ctxt > 2500:
        print(f"  Baseline is elevated ({worst_ctxt:.0f} ctxt/s) though not yet in the")
        print("  range that has broken this board before. Worth watching.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
