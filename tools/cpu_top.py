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
cpu_top.py — who is burning CPU and, more importantly, who is switching context?

Run on the bot.

    sudo ./tools/cpu_top.py           # 10 s sample
    sudo ./tools/cpu_top.py 30
    sudo ./tools/cpu_top.py 10 --threads   # break the worst process into threads

WHY CONTEXT SWITCHES, NOT CPU%
On this board a fault has already been diagnosed once by CPU% and got the wrong
answer: the SBUS driver was doing one read() syscall per byte, the board showed
60%+ IDLE, and ssh was still unusable. The measurement that found it was the
context-switch rate -- 6983-7156/s. top would not have shown you that.

A thread that wakes 2000 times a second to do 50 microseconds of work costs
almost no CPU% and destroys latency for everything else on a single core. That
is the shape this looks for.

WHAT THE COLUMNS MEAN
  cpu%        share of one core, from utime+stime deltas
  vol/s       VOLUNTARY switches: the thread blocked on something -- a read, a
              sleep, a lock. High here means it is waking far too often, which
              is a polling loop or a too-small read.
  invol/s     INVOLUNTARY: the scheduler took the CPU away. High here means CPU
              contention, i.e. something else is the problem, not this thread.

A high vol/s with low cpu% is the signature worth hunting: cheap work, done far
too many times.
"""
import os, sys, time

args = [a for a in sys.argv[1:] if not a.startswith("--")]
THREADS = "--threads" in sys.argv
SECS = float(args[0]) if args else 10.0
HZ = os.sysconf("SC_CLK_TCK")


def read(path):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ""


def cpu_total():
    for line in read("/proc/stat").splitlines():
        if line.startswith("cpu "):
            return sum(int(x) for x in line.split()[1:])
    return 0


def ctxt_of(status_path):
    vol = invol = 0
    for line in read(status_path).splitlines():
        if line.startswith("voluntary_ctxt_switches:"):
            vol = int(line.split()[1])
        elif line.startswith("nonvoluntary_ctxt_switches:"):
            invol = int(line.split()[1])
    return vol, invol


def jiffies_of(stat_text):
    """utime+stime from a /proc/.../stat line, counting fields after the last ')'
    because comm can contain spaces and parentheses."""
    p = stat_text.rfind(")")
    if p < 0:
        return 0
    parts = stat_text[p + 2:].split()
    try:
        return int(parts[11]) + int(parts[12])   # utime, stime (0-indexed here)
    except (IndexError, ValueError):
        return 0


def snapshot(threads=False):
    """key -> (label, jiffies, vol, invol)"""
    out = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        comm = read(f"/proc/{d}/comm").strip()
        if not comm:
            continue
        if threads:
            tdir = f"/proc/{d}/task"
            try:
                tids = os.listdir(tdir)
            except OSError:
                continue
            for t in tids:
                tcomm = read(f"{tdir}/{t}/comm").strip() or comm
                st = read(f"{tdir}/{t}/stat")
                if not st:
                    continue
                v, iv = ctxt_of(f"{tdir}/{t}/status")
                out[f"{d}/{t}"] = (f"{comm}:{tcomm}", jiffies_of(st), v, iv)
        else:
            st = read(f"/proc/{d}/stat")
            if not st:
                continue
            v, iv = ctxt_of(f"/proc/{d}/status")
            out[d] = (comm, jiffies_of(st), v, iv)
    return out


def main():
    if os.geteuid() != 0:
        print("  note: not root — other users' /proc entries are hidden, so the")
        print("        ranking will be incomplete. Use sudo.\n")

    print(f"  sampling {SECS:.0f}s{' (per thread)' if THREADS else ''} ...", flush=True)
    c0, s0 = cpu_total(), snapshot(THREADS)
    time.sleep(SECS)
    c1, s1 = cpu_total(), snapshot(THREADS)

    dtotal = c1 - c0
    rows = []
    tot_vol = tot_invol = 0
    # Anything present at the END but not at the START began mid-sample. Its
    # counters are lifetime-to-date, not a delta over this window, so mixing it
    # into per-second rates would be wrong.
    #
    # But DROPPING it silently is worse, and that is what this used to do. The
    # normal way to deploy here is a VS Code task that syncs, builds and
    # restarts the services -- so measuring right after a deploy gives the bot
    # new PIDs, and every one of them vanished from the table. The tool then
    # printed "1365/s is healthy" about a board whose entire workload it had
    # just discarded. A measurement tool that quietly omits the subject is
    # worse than no tool.
    started_mid = {}
    for k, (label, j, v, iv) in s1.items():
        if k not in s0:
            started_mid.setdefault(label.split(":")[0], 0)
            started_mid[label.split(":")[0]] += 1
            continue
        dj = j - s0[k][1]
        dv = v - s0[k][2]
        div = iv - s0[k][3]
        if dj <= 0 and dv <= 0 and div <= 0:
            continue
        tot_vol += dv
        tot_invol += div
        cpu = 100.0 * dj / dtotal if dtotal else 0.0
        rows.append((dv + div, cpu, dv / SECS, div / SECS, k, label))
    rows.sort(reverse=True)

    print()
    print(f"  {'process' + ('/thread' if THREADS else ''):<30}{'pid':>10}"
          f"{'cpu%':>8}{'vol/s':>9}{'invol/s':>9}")
    print("  " + "-" * 66)
    for _, cpu, v, iv, k, label in rows[:15]:
        print(f"  {label[:29]:<30}{k:>10}{cpu:>8.1f}{v:>9.0f}{iv:>9.0f}")

    print()
    print(f"  totals: {tot_vol/SECS:.0f} voluntary/s + {tot_invol/SECS:.0f} involuntary/s"
          f"  =  {(tot_vol+tot_invol)/SECS:.0f}/s")

    # ── did we actually measure the thing we care about? ────────────────────
    # Ask before judging. The verdict below is a statement about the board, and
    # it is only meaningful if the board's real workload was in the sample.
    WATCH = ("balance_bot", "node", "batt_monitor")
    seen = {label.split(":")[0] for _, _, _, _, _, label in rows}
    absent = [w for w in WATCH if w not in seen]
    restarted = [w for w in WATCH if w in started_mid]

    incomplete = False
    if started_mid:
        print()
        n = sum(started_mid.values())
        who = ", ".join(f"{k} ({v})" for k, v in sorted(started_mid.items(),
                                                       key=lambda x: -x[1])[:6])
        print(f"  NOTE: {n} process/thread(s) started during the sample and are NOT")
        print(f"        in the table or the totals: {who}")
        if restarted:
            incomplete = True
            print(f"        {', '.join(restarted)} restarted mid-sample -- if you just")
            print("        deployed, wait for the services to settle and sample again.")

    if absent:
        incomplete = True
        print()
        print(f"  NOTE: not running at all during this sample: {', '.join(absent)}")

    # ── verdict, judged on ABSOLUTE level, not on change ────────────────────
    # An earlier version of this compared two samples and said "nothing moved
    # much" -- which was true, and useless, because both samples were already
    # in the range that had previously made the board unusable.
    total = (tot_vol + tot_invol) / SECS
    print()
    if incomplete:
        print(f"  NO VERDICT: {total:.0f}/s was measured, but the bot's own processes were")
        print("              missing or restarting, so this number does not describe the")
        print("              board under load. Re-run with everything up and settled:")
        print("                  botss                       confirm what is running")
        print("                  sudo ./tools/cpu_top.py 20 --threads")
        return
    if total > 5000:
        print(f"  VERDICT: {total:.0f} context switches/s is BAD on this board.")
        print("           The SBUS read()-per-byte bug measured 6983-7156/s and made")
        print("           ssh sluggish and the dashboard erratic, with CPU mostly idle.")
        if rows:
            worst = rows[0]
            print(f"           Worst: {worst[5]} ({worst[4]}) at {worst[2]:.0f} vol/s"
                  f" for {worst[1]:.1f}% cpu.")
            if worst[2] > 500 and worst[1] < 20:
                print("           That is a lot of waking for very little work -- a poll")
                print("           loop or an undersized read, not real computation.")
        if not THREADS:
            print("           Re-run with --threads to see which thread inside it.")
    elif total > 2000:
        print(f"  VERDICT: {total:.0f}/s is elevated but not yet in the range that")
        print("           previously broke this board.")
    else:
        print(f"  VERDICT: {total:.0f}/s is healthy for this board.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
