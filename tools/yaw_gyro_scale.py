#!/usr/bin/env python3
"""Fit [yaw] gyro_scale from a ring capture.

The steering loop closes on phi_diff, which only moves in whole encoder counts
of 1.2405 deg. Differencing that staircase is what makes the idle yaw dither
(see README "Yaw Derivative Source"). gyro Z measures the same rotation
continuously; this works out the constant that maps one onto the other.

Procedure:
    1. Kickstand. Turn the bot left and right by hand for ~20 s, at any point
       during a normal run -- gyro_scale does NOT need to be set first.
    2. Dump the ring, then run this on it.

It fits yaw_encRate = k * yaw_psiDot through the origin over the whole run.
yaw_encRate is quantised (1.2405 deg per encoder count) and yaw_psiDot is not,
so no single sample means anything and the fit needs the whole capture. Prints
the scale to enter, sign included.

Older captures without a yaw_psiDot column fall back to yaw_gyroRate, which is
psi_dot already multiplied by whatever gyro_scale was live -- that path needs a
run made with gyro_scale set to something nonzero.

    usage: yaw_gyro_scale.py <ring.csv>
"""
import csv
import io
import math
import sys


def load(path):
    with open(path) as f:
        lines = [l for l in f if not l.startswith('#')]
    return list(csv.DictReader(io.StringIO(''.join(lines))))


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    rows = load(sys.argv[1])
    if not rows:
        sys.exit(f"{sys.argv[1]}: no data rows.")
    if 'yaw_encRate' not in rows[0]:
        sys.exit(f"{sys.argv[1]}: no yaw_encRate column -- this capture "
                 f"predates the gyro-D change.")

    e = [float(r['yaw_encRate']) for r in rows]

    if 'yaw_psiDot' in rows[0]:
        # Raw gyro Z, independent of whatever gyro_scale was live. Preferred.
        g = [float(r['yaw_psiDot']) for r in rows]
    elif 'yaw_gyroRate' in rows[0]:
        # Older capture: psi_dot already multiplied by the live scale. Divide
        # it back out so the answer is an absolute scale, not a correction.
        g = [float(r['yaw_gyroRate']) for r in rows]
        scale_used = None
        with open(sys.argv[1]) as f:
            for line in f:
                if not line.startswith('#'):
                    break
                if 'yaw_gyro_scale=' in line:
                    scale_used = float(line.split('yaw_gyro_scale=')[1].split()[0])
        if not scale_used:
            sys.exit("This capture has no yaw_psiDot column, and gyro_scale was "
                     "0 (or absent) when it was made, so yaw_gyroRate is all "
                     "zeros. Re-capture with current firmware -- it logs the raw "
                     "gyro and needs no special setup.")
        g = [v / scale_used for v in g]
    else:
        sys.exit(f"{sys.argv[1]}: no yaw_psiDot or yaw_gyroRate column.")

    moving = [(a, b) for a, b in zip(g, e) if abs(a) > 2.0]
    if len(moving) < 200:
        sys.exit(f"only {len(moving)} samples with real rotation -- turn the "
                 f"bot by hand through the whole capture and try again.")

    sgg = sum(a * a for a, _ in moving)
    sge = sum(a * b for a, b in moving)
    k = sge / sgg

    # How well it actually fits. A low r means the two signals are not
    # measuring the same rotation and the number below is meaningless.
    ee = sum(b * b for _, b in moving)
    r = sge / math.sqrt(sgg * ee) if sgg and ee else 0.0

    print(f"samples in motion : {len(moving)} of {len(rows)}")
    print(f"correlation       : {r:+.3f}")
    print(f"fitted gyro_scale : {k:+.3f}")
    if abs(r) < 0.8:
        print("\nWEAK FIT. Do not use this. Either the bot barely moved, or "
              "gyro Z is not the axis the wheels are turning about -- check "
              "the IMU mounting before going further.")
    else:
        print(f"\nSet [yaw] gyro_scale = {k:+.3f}")
        if k < 0:
            print("(negative: the IMU is mounted so +gyro Z is -phi_diff. "
                  "Expected, not a mistake.)")


if __name__ == '__main__':
    main()
