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
analyze_tune.py — full tune-quality report from one or more telemetry CSVs.

  ./analyze_tune.py log.csv                    # one run, full report
  ./analyze_tune.py before.csv after.csv       # compare runs side by side
  ./analyze_tune.py *.csv --brief              # one summary line each

What it measures, and why each one matters:

  TRIM       Net travel while trying to hold station. A correctly trimmed robot
             stays put. Creep is the single most misleading fault on this
             machine because the balance integrator hides it — the bot never
             falls, it just wanders, and that reads as a drive problem.

  BALANCE    RMS and peak pitch error, and how much of the run was spent with
             the motors saturated. Saturation means the loop ran out of
             authority: past that point gains are irrelevant, it simply cannot
             push harder.

  RINGING    Dominant oscillation period and amplitude, by autocorrelation.
             A balancing robot always oscillates a little; what matters is
             whether there is one strong periodic mode, which says a gain is
             too high or a damping term is out of phase.

  POSITION   How well station-keeping holds: RMS and peak tick error, time
             inside the tight deadband, and whether position hold is running a
             limit cycle of its own (a slow ~1-3 s mode distinct from balance
             ringing, usually vel_scale_stop or scale_d).

  EFFORT     RMS duty and left/right asymmetry. Persistent asymmetry means one
             side is doing more work — drivetrain drag, or a steering setpoint
             that is not centred.

  HEALTH     Loop rate, missed ticks, falls. A run with a poor loop rate is not
             evidence of anything; the numbers above are only as good as the
             sampling behind them.

Every metric prints with a verdict (ok / warn / bad) against a stated threshold,
so the report says what to do rather than leaving you to interpret it.
"""

import argparse
import math
import os
import sys

# ── thresholds ──────────────────────────────────────────────────────
# Deliberately explicit. Each is printed next to the value it judges so the
# report can be argued with rather than trusted blindly.
TH = {
    "drift_ok":        0.30,   # ticks/100ms, mean |velocity| while holding
    "drift_bad":       1.00,
    "theta_rms_ok":    1.50,   # deg RMS pitch error
    "theta_rms_bad":   3.00,
    "theta_peak_ok":   6.00,   # deg, worst excursion
    "theta_peak_bad":  12.00,
    "sat_ok":          2.0,    # % of samples with |duty| > 0.95
    "sat_bad":         10.0,
    "ring_amp_ok":     1.0,    # deg, amplitude of dominant mode
    "ring_amp_bad":    3.0,
    "pos_rms_ok":      60.0,   # ticks RMS position error
    "pos_rms_bad":     200.0,
    "asym_ok":         0.05,   # mean |L-R| duty
    "asym_bad":        0.15,
    "loop_ok":         2.0,    # % of samples below 95 Hz
    "loop_bad":        10.0,
}

# Wheel 4.75 in = 120.65 mm dia, 145.1 counts per wheel revolution.
# The /2 matters: robot.c does `enc_pos = left_ticks + right_ticks`, so one
# unit of enc_pos is half a wheel-tick of travel. Without it every distance
# in this report reads 2x high -- and with the old 155 mm diameter it read
# 2.57x high, which is why 5 inches of real wander printed as 12.6.
MM_PER_TICK = 120.65 * math.pi / (2 * 145.1)


def verdict(value, ok, bad, lower_is_better=True):
    if not lower_is_better:
        ok, bad = -ok, -bad
        value = -value
    if value <= ok:
        return "ok"
    if value >= bad:
        return "BAD"
    return "warn"


MARK = {"ok": "  ok ", " warn": "warn ", "warn": "warn ", "BAD": "BAD  "}


# ── csv ─────────────────────────────────────────────────────────────

def read_csv(path):
    rows, hdr, pre = [], None, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                pre.append(line.rstrip())
                continue
            parts = line.rstrip("\n").split(",")
            if hdr is None:
                hdr = parts
                continue
            if len(parts) != len(hdr):
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue
    return pre, hdr, rows


# Column prefixes have been through three generations. All three are still
# readable, because there is a directory of logs in each and they are all
# perfectly good data:
#
#   D1 -> balance -> pitch      d1_  ->  bal_  ->  pit_
#   D2 -> position              d2_  ->  pos_
#   D3 -> steering -> yaw       d3_  ->  str_  ->  yaw_
LEGACY_PREFIX = {"pit_": ["bal_", "d1_"], "pos_": ["d2_"], "yaw_": ["str_", "d3_"]}


def aliases(name):
    """Every older spelling of a column name, newest first."""
    for new, olds in LEGACY_PREFIX.items():
        if name.startswith(new):
            return [old + name[len(new):] for old in olds]
    return []


class Log:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.pre, self.hdr, self.rows = read_csv(path)
        self.idx = {k: n for n, k in enumerate(self.hdr)} if self.hdr else {}
        self.why = ""

    def ok(self):
        if not self.rows:
            self.why = "no data rows"
            return False
        if any(k in self.idx for k in
               ("pit_measurement", "bal_measurement", "d1_measurement")):
            return True
        self.why = ("no pitch column (need pit_measurement, or the older "
                    "bal_measurement / d1_measurement); found: "
                    + ", ".join(self.hdr[:6]) + " ...")
        return False

    def c(self, name, default=None):
        n = self.idx.get(name)
        if n is None:
            for legacy in aliases(name):
                n = self.idx.get(legacy)
                if n is not None:
                    break
        if n is None:
            return default
        return [r[n] for r in self.rows]

    def preamble(self, key):
        for line in self.pre:
            if key in line:
                return line.lstrip("# ").rstrip()
        return None


# ── statistics ──────────────────────────────────────────────────────

def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else 0.0


def mean(v):
    return sum(v) / len(v) if v else 0.0


def autocorr_period(sig, dt, lo=0.05, hi=5.0):
    """Dominant period by autocorrelation, in seconds, or None.

    Used instead of an FFT so the tool has no numpy dependency and so an
    irregular sample interval degrades gracefully. Returns the lag of the first
    clear peak after the autocorrelation first goes negative — that skips the
    trivial peak at zero lag without needing to window or detrend heavily.
    """
    n = len(sig)
    if n < 64 or dt <= 0:
        return None, 0.0
    m = mean(sig)
    x = [v - m for v in sig]
    denom = sum(v * v for v in x)
    if denom <= 0:
        return None, 0.0

    lo_lag = max(1, int(lo / dt))
    hi_lag = min(n // 2, int(hi / dt))
    if hi_lag <= lo_lag:
        return None, 0.0

    seen_negative = False
    best_lag, best_val = None, 0.0
    for lag in range(1, hi_lag):
        s = sum(x[k] * x[k + lag] for k in range(n - lag)) / denom
        if s < 0:
            seen_negative = True
            continue
        if seen_negative and lag >= lo_lag and s > best_val:
            best_val, best_lag = s, lag
    if best_lag is None:
        return None, 0.0
    # amplitude of that mode, approximated by the RMS of the signal scaled by
    # how much of its energy the mode accounts for
    return best_lag * dt, rms(x) * math.sqrt(max(best_val, 0.0)) * math.sqrt(2)


def analyse(log, max_theta=8.0):
    """Everything the report needs, as a flat dict."""
    r = {"name": log.name, "n": len(log.rows)}

    t = log.c("t") or []
    th = log.c("pit_measurement") or []
    sp = log.c("pit_setpoint") or []
    ev = log.c("pos_encVel") or []
    ep = log.c("pos_encPos") or []
    ee = log.c("pos_encError") or []
    ld = log.c("mot_leftDuty") or []
    rd = log.c("mot_rightDuty") or []

    r["dur"] = (t[-1] - t[0]) if len(t) > 1 else 0.0
    r["dt"] = r["dur"] / (len(t) - 1) if len(t) > 1 else 0.0

    # balancing samples only — falls and being held would poison everything
    keep = [k for k in range(len(th)) if abs(th[k]) <= max_theta]
    r["keep"] = len(keep)
    r["dropped_pct"] = 100.0 * (len(th) - len(keep)) / len(th) if th else 0.0
    if not keep:
        return r

    thk = [th[k] for k in keep]
    err = [sp[k] - th[k] for k in keep] if sp else []

    # TRIM
    r["net_ticks"] = (ep[-1] - ep[0]) if ep else 0.0
    r["net_m"] = r["net_ticks"] * MM_PER_TICK / 1000.0
    r["drift"] = mean([ev[k] for k in keep]) if ev else 0.0
    r["drift_abs"] = abs(r["drift"])

    # BALANCE
    r["theta_rms"] = rms(err) if err else rms(thk)
    r["theta_peak"] = max(abs(x) for x in (err if err else thk))
    r["theta_mean"] = mean(thk)
    if ld and rd:
        sat = sum(1 for k in keep if abs(ld[k]) > 0.95 or abs(rd[k]) > 0.95)
        r["sat_pct"] = 100.0 * sat / len(keep)
        r["duty_rms"] = rms([(ld[k] + rd[k]) / 2 for k in keep])
        r["asym"] = mean([abs(ld[k] - rd[k]) for k in keep])
    else:
        r["sat_pct"] = r["duty_rms"] = r["asym"] = 0.0

    # RINGING — on pitch
    per, amp = autocorr_period(thk, r["dt"])
    r["ring_period"], r["ring_amp"] = per, amp

    # POSITION
    if ee:
        eek = [ee[k] for k in keep]
        r["pos_rms"] = rms(eek)
        r["pos_peak"] = max(abs(x) for x in eek)
        # Read the bot's real deadband from the log header when it is there.
        # This used to be a hardcoded 2, which silently measured the wrong
        # thing the moment pos_deadband became tunable.
        db = log.c("pos_deadband", 2)
        try:
            db = int(float(db))
        except (TypeError, ValueError):
            db = 2
        r["pos_deadband"] = db
        # Peak-to-peak travel: the swing you can SEE the machine make.
        # RMS is the honest statistic for comparing tunes, but it runs ~4x
        # smaller than the visible wander and reads as success when the bot is
        # plainly sliding around. Report both or the report argues with the eye.
        r["pos_p2p"] = (max(eek) - min(eek)) if eek else 0.0
        r["pos_tight_pct"] = 100.0 * sum(1 for x in eek if abs(x) < db) / len(eek)
        pper, pamp = autocorr_period(eek, r["dt"], lo=0.5, hi=8.0)
        r["pos_period"], r["pos_amp"] = pper, pamp
    else:
        r["pos_rms"] = r["pos_peak"] = r["pos_tight_pct"] = r["pos_p2p"] = 0.0
        r["pos_deadband"] = 2
        r["pos_period"] = None
        r["pos_amp"] = 0.0

    # HEALTH
    loop = log.preamble("loop_hz")
    r["loop_line"] = loop
    r["below95"] = 0.0
    if loop and "below_95Hz=" in loop and "samples=" in loop:
        try:
            b = float(loop.split("below_95Hz=")[1].split()[0])
            s = float(loop.split("samples=")[1].split()[0])
            r["below95"] = 100.0 * b / s if s else 0.0
        except (ValueError, IndexError):
            pass
    r["falls"] = sum(1 for x in th if abs(x) > 14.0)
    return r


# ── reporting ───────────────────────────────────────────────────────

def row(label, value, fmt, v, note=""):
    mark = {"ok": "ok  ", "warn": "warn", "BAD": "BAD "}[v]
    print(f"    [{mark}] {label:<26} {fmt % value:>10}   {note}")


def report(r):
    print(f"\n{'=' * 72}")
    print(f"  {r['name']}")
    print(f"{'=' * 72}")
    if r.get("keep", 0) == 0:
        print("  No balancing samples — nothing to analyse.")
        return

    print(f"  {r['n']} samples over {r['dur']:.1f}s "
          f"({r['keep']} balancing, {r['dropped_pct']:.0f}% dropped as falls/held)")

    print("\n  TRIM — does it hold station?")
    v = verdict(r["drift_abs"], TH["drift_ok"], TH["drift_bad"])
    row("mean drift", r["drift"], "%+.2f", v, f"ticks/100ms (ok <{TH['drift_ok']})")
    row("net travel", r["net_m"], "%+.2f", v, f"m in {r['dur']:.0f}s")

    print("\n  BALANCE — how tightly does it track upright?")
    v = verdict(r["theta_rms"], TH["theta_rms_ok"], TH["theta_rms_bad"])
    row("pitch error RMS", r["theta_rms"], "%.2f", v, f"deg (ok <{TH['theta_rms_ok']})")
    v = verdict(r["theta_peak"], TH["theta_peak_ok"], TH["theta_peak_bad"])
    row("pitch error peak", r["theta_peak"], "%.2f", v, f"deg (ok <{TH['theta_peak_ok']})")
    v = verdict(r["sat_pct"], TH["sat_ok"], TH["sat_bad"])
    row("motor saturation", r["sat_pct"], "%.1f", v, f"% of samples (ok <{TH['sat_ok']}%)")

    print("\n  RINGING — is there one dominant oscillation?")
    if r["ring_period"]:
        v = verdict(r["ring_amp"], TH["ring_amp_ok"], TH["ring_amp_bad"])
        row("dominant period", r["ring_period"], "%.2f", v, "s")
        row("amplitude", r["ring_amp"], "%.2f", v,
            f"deg (ok <{TH['ring_amp_ok']})")
    else:
        print("    [ok  ] no dominant periodic mode found")

    print("\n  POSITION HOLD — station-keeping quality")
    v = verdict(r["pos_rms"], TH["pos_rms_ok"], TH["pos_rms_bad"])
    row("position error RMS", r["pos_rms"], "%.0f", v,
        f"ticks = {r['pos_rms']*MM_PER_TICK:.0f} mm (ok <{TH['pos_rms_ok']:.0f})")
    row("position error peak", r["pos_peak"], "%.0f", v,
        f"ticks = {r['pos_peak']*MM_PER_TICK/1000:.2f} m")
    print(f"    [    ] {'travel peak-to-peak':<26} {r['pos_p2p']:>9.0f}   "
          f"ticks = {r['pos_p2p']*MM_PER_TICK/25.4:.1f} in  <- the wander you can see")
    print(f"    [    ] {'time inside deadband':<26} {r['pos_tight_pct']:>9.0f}%   "
          f"(|err| < {r['pos_deadband']} ticks = "
          f"{r['pos_deadband']*MM_PER_TICK:.0f} mm)")
    if r["pos_period"]:
        print(f"    [    ] {'limit cycle':<26} {r['pos_period']:>9.2f}s   "
              f"amplitude {r['pos_amp']:.0f} ticks = {r['pos_amp']*MM_PER_TICK:.0f} mm")

    print("\n  EFFORT — how hard is it working?")
    row("duty RMS", r["duty_rms"], "%.3f", "ok", "of full scale")
    v = verdict(r["asym"], TH["asym_ok"], TH["asym_bad"])
    row("left/right asymmetry", r["asym"], "%.3f", v, f"duty (ok <{TH['asym_ok']})")

    print("\n  HEALTH — is this run trustworthy?")
    v = verdict(r["below95"], TH["loop_ok"], TH["loop_bad"])
    row("loop ticks below 95Hz", r["below95"], "%.1f", v, f"% (ok <{TH['loop_ok']}%)")
    row("samples beyond 14 deg", r["falls"], "%d", "ok" if r["falls"] == 0 else "warn",
        "fall-detection territory")

    advise(r)


def advise(r):
    """Turn the numbers into the next thing to try, in priority order."""
    todo = []
    if r["below95"] >= TH["loop_bad"]:
        todo.append("Loop rate is poor. Fix that first — every number above is "
                    "measured through it. Raise POS_VEL_PERIOD_MS or find what "
                    "is stealing CPU.")
    if r["drift_abs"] > TH["drift_ok"]:
        d = "-0.3" if r["drift"] > 0 else "+0.3"
        todo.append(f"Creeping {'forward' if r['drift'] > 0 else 'backward'}. "
                    f"Adjust balance trim by {d} deg and re-log before touching "
                    f"any gain — creep masquerades as every other fault.")
    if r["sat_pct"] >= TH["sat_bad"]:
        todo.append(f"Motors saturated on {r['sat_pct']:.0f}% of samples. The loop "
                    "is out of authority; raising gains cannot help. Reduce "
                    "commanded lean, or check for mechanical drag.")
    if r["ring_period"] and r["ring_amp"] >= TH["ring_amp_bad"]:
        todo.append(f"Strong {r['ring_period']:.2f}s oscillation at "
                    f"{r['ring_amp']:.1f} deg. Fast (<0.5s) usually means kp or kd "
                    "too high; slow (>1s) usually means position hold fighting "
                    "balance — look at vel_scale_stop and scale_d.")
    if r["pos_rms"] >= TH["pos_rms_bad"]:
        todo.append(f"Position error RMS {r['pos_rms']:.0f} ticks "
                    f"({r['pos_rms']*MM_PER_TICK/1000:.2f} m). Station-keeping is "
                    "loose — scale_d, or back_to_spot is off.")
    if r["asym"] >= TH["asym_bad"]:
        todo.append(f"Left/right duty differs by {r['asym']:.3f} on average. One "
                    "side is working harder: drivetrain drag, or the steering "
                    "setpoint is not centred.")

    print("\n  WHAT TO DO NEXT")
    if not todo:
        todo = ["Nothing flagged. This is a good tune -- run 'make save-config' "
                "and commit config/machine/ before changing anything."]
        for line in wrap(todo[0], 66):
            print(f"    {line}")
        return
    for k, item in enumerate(todo, 1):
        lines = wrap(item, 66)
        print(f"    {k}. {lines[0]}")
        for line in lines[1:]:
            print(f"       {line}")


def wrap(text, width):
    """Minimal word wrap -- textwrap would do, but keeping this dependency-free
    so the script runs on any python3 without imports beyond the stdlib basics."""
    out, cur = [], ""
    for word in text.split():
        if cur and len(cur) + 1 + len(word) > width:
            out.append(cur)
            cur = word
        else:
            cur = f"{cur} {word}".strip()
    if cur:
        out.append(cur)
    return out or [""]


def brief(r):
    if r.get("keep", 0) == 0:
        print(f"  {r['name']:<34} (no balancing samples)")
        return
    flags = []
    if r["drift_abs"] > TH["drift_ok"]:
        flags.append("creep")
    if r["sat_pct"] >= TH["sat_bad"]:
        flags.append("saturated")
    if r["ring_period"] and r["ring_amp"] >= TH["ring_amp_bad"]:
        flags.append("ringing")
    if r["below95"] >= TH["loop_bad"]:
        flags.append("slow-loop")
    print(f"  {r['name']:<34} {r['dur']:5.0f}s  drift {r['drift']:+5.2f}  "
          f"theta_rms {r['theta_rms']:4.2f}  sat {r['sat_pct']:4.1f}%  "
          f"pos_rms {r['pos_rms']:5.0f}  {'/'.join(flags) if flags else 'clean'}")


def compare(results):
    print(f"\n{'=' * 96}")
    print("  COMPARISON")
    print(f"{'=' * 96}")
    cols = [("drift", "drift", "%+7.2f"), ("theta_rms", "pitch RMS", "%8.2f"),
            ("theta_peak", "pitch peak", "%9.2f"), ("sat_pct", "sat %", "%6.1f"),
            ("pos_rms", "pos RMS", "%8.0f"), ("asym", "asym", "%7.3f")]
    print(f"  {'run':<30}" + "".join(f"{lbl:>11}" for _, lbl, _ in cols))
    for r in results:
        if r.get("keep", 0) == 0:
            continue
        line = f"  {r['name'][:29]:<30}"
        for key, _, fmt in cols:
            line += f"{fmt % r[key]:>11}"
        print(line)
    # call out what improved
    if len(results) >= 2:
        a, b = results[0], results[-1]
        if a.get("keep") and b.get("keep"):
            print(f"\n  {a['name']}  ->  {b['name']}")
            for key, lbl, _ in cols:
                d = b[key] - a[key]
                if abs(d) < 1e-9:
                    continue
                better = abs(b[key]) < abs(a[key])
                print(f"    {lbl:<12} {a[key]:+8.2f} -> {b[key]:+8.2f}  "
                      f"({d:+.2f}, {'better' if better else 'worse'})")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="one or more telemetry CSVs")
    ap.add_argument("--brief", action="store_true", help="one line per run")
    ap.add_argument("--max-theta", type=float, default=8.0,
                    help="pitch beyond this is a fall, not balancing (default 8)")
    a = ap.parse_args()

    results = []
    for p in a.csv:
        log = Log(p)
        if not log.ok():
            print(f"  {os.path.basename(p)}: {getattr(log, 'why', 'unusable')}",
                  file=sys.stderr)
            continue
        results.append(analyse(log, a.max_theta))

    if not results:
        return 1
    if a.brief:
        print()
        for r in results:
            brief(r)
    else:
        for r in results:
            report(r)
    if len(results) > 1:
        compare(results)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
