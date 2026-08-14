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

So measure the drift instead of the pose. Bin the balancing samples by pitch,
average wheel velocity within each bin, and find where that curve crosses zero.
That angle is where the robot does not accelerate: the true balance point, and
the value the balance trim should be set to.

Binning matters. A point-wise fit of velocity against angle is mostly noise,
because velocity is the INTEGRAL of angle error rather than proportional to it —
the two run about 90 degrees out of phase. Averaging within a bin cancels that.

The curve may slope up or down depending on IMU mounting and motor/encoder
polarity; only monotonicity and the crossing matter.

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


def col(rows, idx, name):
    if name not in idx:
        return None
    n = idx[name]
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
    if theta is None or vel is None:
        print("need bal_measurement and pos_encVel columns; found:\n  " +
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

    same = all(m > 0 for _, m, _ in curve) or all(m < 0 for _, m, _ in curve)

    # A believable equilibrium needs more than one bin flipping sign: require a
    # run of RUN bins on one side of zero and RUN on the other. A single-bin dip
    # between two same-signed bins is sensor noise, and treating it as a crossing
    # produced a confidently wrong answer (-4.88 deg) on the first real log.
    #
    # The curve may slope EITHER way. Whether leaning forward reads as +theta,
    # and whether forward motion reads as +encVel, depend on IMU mounting and on
    # pol_*/enc_pol_* -- on this robot all four are -1 and the curve slopes down.
    # An earlier version demanded an increasing curve and threw away a clean
    # r=-0.91 signal. What matters is monotonic and crossing zero, not the sign.
    RUN = 2
    cross = None
    for k in range(len(curve) - 1):
        y0, y1 = curve[k][1], curve[k + 1][1]
        if not (y0 <= 0 <= y1 or y0 >= 0 >= y1):
            continue
        below = [curve[j][1] for j in range(max(0, k - RUN + 1), k + 1)]
        above = [curve[j][1] for j in range(k + 1, min(len(curve), k + 1 + RUN))]
        if len(below) < RUN or len(above) < RUN:
            continue
        rising = all(v <= 0 for v in below) and all(v >= 0 for v in above)
        falling = all(v >= 0 for v in below) and all(v <= 0 for v in above)
        if rising or falling:
            f = 0 if y1 == y0 else (0 - y0) / (y1 - y0)
            cross = curve[k][0] + f * (curve[k + 1][0] - curve[k][0])
            break

    # The crossing only means anything if the curve is broadly increasing. If
    # velocity does not rise with pitch, there is no single equilibrium in this
    # data and any crossing is an artefact -- on the first real log this scored
    # r=0.35 and produced a crossing that contradicted the log's own mean drift.
    bx = [b for b, _, _ in curve]
    by = [m for _, m, _ in curve]
    _, _, r_curve = linfit(bx, by)
    strong = r_curve is not None and abs(r_curve) > 0.5
    print(f"\n  binned curve trend: r = {r_curve:+.2f} "
          f"({'usable' if strong else 'too noisy to locate an equilibrium'})")

    if cross is not None and strong:
        if abs(cross) < 0.3:
            print(f"\n  Equilibrium at {cross:+.2f} deg — already trimmed, leave it alone.")
        else:
            print(f"\n  EQUILIBRIUM: {cross:+.2f} deg   (velocity crosses zero here,")
            print(f"  with a monotonic curve either side, so this is trustworthy.)")
            print(f"  Apply {cross:+.2f} deg to the balance trim, then re-log to confirm.")
        return 0

    print()
    if abs(mean_drift) < 0.3:
        print("  Mean drift is small, but the curve is too noisy to confirm the")
        print("  equilibrium. Re-log before concluding it is trimmed -- a symmetric")
        print("  swing about a WRONG balance point also averages to zero drift.")
        return 0

    direction = "MORE NEGATIVE" if mean_drift > 0 else "MORE POSITIVE"
    print(f"  The bot creeps {'forward' if mean_drift > 0 else 'backward'} persistently.")
    print(f"  Balance trim needs to go {direction}.")

    if same:
        print()
        print("  Every pitch bin drifts the same way, so the balance point is OUTSIDE")
        print("  the range of angles in this log. The magnitude cannot be read off")
        print("  this data -- only the direction.")
        step = -0.5 if mean_drift > 0 else 0.5
        print(f"\n  Apply {step:+.1f} deg, record another log, and run this again.")
        print("  Iterate until the mean drift falls under +/-0.3. Do NOT try to")
        print("  jump straight to a computed value; nothing here supports one.")
    else:
        step = -0.5 if mean_drift > 0 else 0.5
        print(f"\n  The curve is not monotonic, so there is no single equilibrium to")
        print(f"  read off it. Direction is reliable; magnitude is not.")
        if cross is not None:
            print(f"  (It does cross zero at {cross:+.2f} deg, but with r={r_curve:+.2f} that")
            print(f"   crossing is noise -- note it contradicts the drift above.)")
        print(f"\n  Apply {step:+.1f} deg, re-log, and run this again. Iterate until the")
        print("  mean drift is under +/-0.3 and the trend firms up above r=0.5.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
