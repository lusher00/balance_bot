#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
trim_from_log.py — compute the balance trim from a telemetry CSV.

Why this exists: eyeballing "upright" does not work, and it fails in a way that
feels like success.

The balance point is where the centre of mass sits over the tyre contact patch.
You cannot see that — you see the chassis, and with the battery, RoboClaw and
BeagleBone distributed unevenly the CoM is not on the geometric centreline. An
error of a few degrees is invisible to the eye and to the hand.

Worse, the balance PID's integrator hides it. With ki > 0 the integrator winds
up to compensate, so the robot balances happily at the wrong angle. It never
falls, so nothing feels wrong. The error appears only as a slow creep, which
reads as a drive problem rather than a trim problem.

So measure the drift instead of the pose. The verdict is NET POSITION DRIFT: a
correctly trimmed robot holds its ground. Anything else creeps, and the creep
rate tells you which way to move the trim.

  trim  0.00  ->  +1.85 m over 62 s   (creeping forward, badly out)
  trim -0.60  ->  -0.04 m over 81 s   (trimmed)

DO NOT try to compute the balance angle from the relationship between pitch and
wheel velocity. Under closed-loop control the balance PID regulates theta, so
the pitch variation in a log is oscillation, not exploration of an equilibrium.
Binning velocity against pitch recovers the PHASE relationship between them —
which looks beautifully clean (this robot scores r = -0.91) and whose zero
crossing is a phase artefact, nothing to do with the balance point. An earlier
version of this tool did exactly that and confidently recommended -2.44 deg for
a robot that was already trimmed at -0.60. The bot took off.

The curve is still printed, because its shape is a useful sanity check on
whether the robot was genuinely balancing. It is a diagnostic, not an estimator.

  ./trim_from_log.py bbot_1786725755805.csv
  ./trim_from_log.py log.csv --max-theta 8      # tighten the balancing filter

Take two or three logs and check the answers agree to a few tenths before you
trust one. A log where the robot fell, was held, or sat against something will
skew the fit — the script reports the diagnostics you need to spot that.
"""

import argparse
import sys


def read_csv(path):
    """Reader for the dashboard's export: '#' preamble, then a header row."""
    rows, hdr, preamble = [], None, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                preamble.append(line.rstrip())
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
    return preamble, hdr, rows


# Logs from before the controller rename use d1_/d2_/d3_ for bal_/pos_/str_.
LEGACY_PREFIX = {"bal_": "d1_", "pos_": "d2_", "str_": "d3_"}


def col(rows, idx, name):
    n = idx.get(name)
    if n is None:
        for new, old in LEGACY_PREFIX.items():
            if name.startswith(new):
                n = idx.get(old + name[len(new):])
                break
    if n is None:
        return None
    return [r[n] for r in rows]


def linfit(x, y):
    n = len(x)
    mx, my = sum(x) / n, sum(y) / n
    den = sum((v - mx) ** 2 for v in x)
    if den == 0:
        return None, None, None
    slope = sum((x[k] - mx) * (y[k] - my) for k in range(n)) / den
    icept = my - slope * mx
    # correlation, to say how much the fit deserves to be believed
    sy = sum((v - my) ** 2 for v in y)
    r = 0.0 if sy == 0 else (slope * (den ** 0.5) / (sy ** 0.5))
    return slope, icept, r


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="telemetry CSV exported from the dashboard")
    ap.add_argument("--max-theta", type=float, default=8.0,
                    help="ignore samples beyond this |pitch|, which are falls "
                         "or the bot resting on something (default: 8)")
    ap.add_argument("--min-samples", type=int, default=50,
                    help="refuse to fit fewer than this many samples (default: 50)")
    a = ap.parse_args()

    preamble, hdr, rows = read_csv(a.csv)
    if not rows:
        print("no data rows found — is this a dashboard CSV export?", file=sys.stderr)
        return 1
    idx = {k: n for n, k in enumerate(hdr)}

    theta = col(rows, idx, "bal_measurement")
    vel = col(rows, idx, "pos_encVel")
    pos = col(rows, idx, "pos_encPos")
    tcol = col(rows, idx, "t")
    if theta is None or vel is None:
        print("need a pitch and a velocity column (bal_measurement/pos_encVel,\n"
              "or the older d1_measurement/d2_encVel). Found:\n  " +
              ", ".join(hdr), file=sys.stderr)
        return 1

    for line in preamble:
        if "bal_gains" in line or "max_correction" in line or "loop_hz" in line:
            print("  " + line)
    print()

    total = len(rows)
    keep = [n for n in range(total) if abs(theta[n]) <= a.max_theta]
    print(f"  {total} samples, {len(keep)} within +/-{a.max_theta:g} deg of upright")
    if len(keep) < a.min_samples:
        print(f"\n  Only {len(keep)} usable samples. This log is mostly not balancing —"
              f"\n  record one where it stays up for 20-30 seconds.", file=sys.stderr)
        return 1

    dropped = total - len(keep)
    if dropped > total * 0.4:
        print(f"  WARNING: dropped {dropped} samples ({100*dropped/total:.0f}%) as "
              f"falls/held.\n           Treat this result with suspicion.")

    th = [theta[n] for n in keep]
    ev = [vel[n] for n in keep]

    # Bin by pitch and average the velocity in each bin. A point-wise fit of
    # velocity against angle is mostly noise, because velocity is the INTEGRAL
    # of angle error, not proportional to it -- the two are ~90 deg out of
    # phase. Averaging within a bin cancels that phase and leaves the trend.
    bins = {}
    for k in range(len(th)):
        bins.setdefault(round(th[k] * 2) / 2, []).append(ev[k])
    curve = [(b, sum(v) / len(v), len(v)) for b, v in sorted(bins.items()) if len(v) >= 8]

    mean_drift = sum(ev) / len(ev)
    print(f"\n  mean pitch {sum(th)/len(th):+.2f} deg")
    print(f"  mean drift {mean_drift:+.2f} ticks/100ms   "
          f"({'FORWARD' if mean_drift > 0 else 'BACKWARD' if mean_drift < 0 else 'none'})")

    if not curve:
        print("\n  Not enough samples in any pitch bin to say anything. Record longer.")
        return 1

    print(f"\n  mean drift per pitch bin ({len(curve)} bins with >=8 samples):")
    for b, m, n in curve:
        print(f"    {b:+6.1f} deg  n={n:4d}  {m:+7.2f} {'#' * min(int(abs(m) * 2), 40)}")

    # ── verdict: net position drift ──────────────────────────────────
    # This, and only this, decides the trim. See the module docstring for why
    # the pitch/velocity curve above must not be used to compute a number.
    net = pos[-1] - pos[0]
    dur = tcol[-1] - tcol[0]
    mm_per_tick = 155.0 * 3.14159 / 145.1
    print(f"\n  net travel {net:+.0f} ticks = {net*mm_per_tick/1000:+.2f} m "
          f"over {dur:.1f}s  ({net/dur*mm_per_tick/1000:+.3f} m/s)")

    DEADBAND = 0.30           # ticks/100ms considered "not creeping"
    print()
    if abs(mean_drift) <= DEADBAND:
        print(f"  TRIMMED. Mean drift {mean_drift:+.2f} is inside +/-{DEADBAND},")
        print(f"  and it travelled {abs(net*mm_per_tick/1000):.2f} m in {dur:.0f}s.")
        print("  Leave the trim alone. Do not chase the curve above.")
        return 0

    # Scale the suggested step by how hard it is creeping, but keep it small --
    # converging in three careful steps beats one confident overshoot.
    step = 0.5 if abs(mean_drift) > 1.5 else 0.3
    step = -step if mean_drift > 0 else step
    print(f"  CREEPING {'forward' if mean_drift > 0 else 'backward'} at "
          f"{mean_drift:+.2f} ticks/100ms ({net/dur*mm_per_tick/1000:+.3f} m/s).")
    print(f"\n  Adjust the balance trim by {step:+.1f} deg, re-log 30s, run this again.")
    print(f"  Converged when the drift is inside +/-{DEADBAND}. It usually takes")
    print("  two or three steps; stop as soon as it says TRIMMED.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
