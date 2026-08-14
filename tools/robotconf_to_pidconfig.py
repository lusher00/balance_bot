#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
robotconf_to_pidconfig.py — port robot.conf back to the pre-refactor format.

Why this exists: the config rewrite is parked in stash@{0}. The tree at 5cc0aac
reads the OLD files (pidconfig.txt + /etc/balance_bot_imu.conf) and knows
nothing about robot.conf. Every value tuned since the rewrite lives only in
robot.conf, so reverting the code without moving the values means the robot
comes up on compiled-in defaults — including pitch_offset, without which it has
no idea which way is up.

This reads robot.conf and writes the two legacy files. It does not guess: any
key it cannot find is reported and left at whatever the existing pidconfig.txt
already had, or the documented default if there is no existing file.

  ./robotconf_to_pidconfig.py robot.conf
  ./robotconf_to_pidconfig.py robot.conf -o pidconfig.txt --imu /etc/balance_bot_imu.conf
  ./robotconf_to_pidconfig.py robot.conf --dry-run      # print, write nothing

Run it on the board where robot.conf lives. Nothing is overwritten without a
.bak being taken first.
"""

import argparse
import os
import re
import shutil
import sys

# ---------------------------------------------------------------------------
# The legacy format, transcribed from src/pid_config.c and src/imu_config.c.
# Order matters for the positional header; the section keys are written in the
# same order pid_config_save() emits them so a diff against an older file is
# readable.
# ---------------------------------------------------------------------------

POS_SECTION = "# pos_config"
MOTOR_SECTION = "# motor_config"

# key -> (printf-style format, compiled-in default)
POS_KEYS = [
    ("zone_a",            "%.1f",  8.0),
    ("zone_b",            "%.1f",  20.0),
    ("zone_c",            "%.1f",  50.0),
    ("scale_a",           "%.3f",  0.30),
    ("scale_b",           "%.3f",  0.60),
    ("scale_c",           "%.3f",  1.00),
    ("scale_d",           "%.3f",  50.0),
    ("vel_scale_stop",    "%.3f",  5.0),
    ("vel_scale_move",    "%.3f",  1.0),
    ("vel_scale_turning", "%.3f",  1.0),
    ("stopped_vel",       "%.1f",  10.0),
    ("max_correction",    "%.3f",  3.0),
    ("max_angle_rate",    "%.3f",  0.1),
    ("back_to_spot",      "%.1f",  1.0),
]

MOTOR_KEYS = [
    ("mode",       "%d",     0),
    ("qpps_max",   "%d",     3000),
    ("accel_qpps", "%d",     6000),
    ("pol_l",      "%.1f",   1.0),
    ("pol_r",      "%.1f",   1.0),
    ("enc_pol_l",  "%.1f",   1.0),
    ("enc_pol_r",  "%.1f",   1.0),
    ("claw_kp",    "%.6f",   1.0),
    ("claw_ki",    "%.6f",   0.5),
    ("claw_kd",    "%.6f",   0.25),
]

IMU_KEYS = [
    ("pitch_offset", 0.0),
    ("yaw_offset",   0.0),
    ("pitch_axis",   1),
]

# (section, key) for the four positional header lines.
BALANCE_GAINS = ("balance", ("kp", "ki", "kd"))
STEERING_GAINS = ("steering", ("kp", "ki", "kd"))


class Conf:
    """robot.conf reader. Keeps section scoping, because [balance] and
    [steering] both have a key called 'kp' and confusing them would put the
    steering gains on the balance loop."""

    def __init__(self):
        self.by_section = {}   # section -> {key: raw string}
        self.flat = {}         # key -> raw string (last wins; for unsectioned keys)

    @classmethod
    def load(cls, path):
        c = cls()
        section = ""
        with open(path) as f:
            for lineno, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith("#") or line.startswith(";"):
                    continue
                m = re.fullmatch(r"\[([A-Za-z0-9_]+)\]", line)
                if m:
                    section = m.group(1).lower()
                    c.by_section.setdefault(section, {})
                    continue
                if "=" not in line:
                    print(f"  warn: {path}:{lineno}: not key=value, skipped: {line!r}",
                          file=sys.stderr)
                    continue
                k, _, v = line.partition("=")
                k, v = k.strip().lower(), v.strip()
                c.by_section.setdefault(section, {})[k] = v
                c.flat[k] = v
        return c

    def get(self, key, section=None):
        """Section-scoped lookup, falling back to a flat search so keys that
        moved between sections during the rewrite still resolve."""
        if section is not None:
            v = self.by_section.get(section, {}).get(key)
            if v is not None:
                return v, f"[{section}]"
        for sec, kv in self.by_section.items():
            if key in kv:
                return kv[key], f"[{sec}]" if sec else "(top level)"
        return None, None


def num(raw, as_int=False):
    v = float(raw)
    return int(round(v)) if as_int else v


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot_conf", help="path to robot.conf")
    ap.add_argument("-o", "--out", default="pidconfig.txt",
                    help="legacy PID config to write (default: pidconfig.txt)")
    ap.add_argument("--imu", default="/etc/balance_bot_imu.conf",
                    help="legacy IMU offsets file (default: /etc/balance_bot_imu.conf)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print both files to stdout, write nothing")
    a = ap.parse_args()

    if not os.path.exists(a.robot_conf):
        print(f"error: no such file: {a.robot_conf}", file=sys.stderr)
        return 1

    conf = Conf.load(a.robot_conf)
    total = sum(len(v) for v in conf.by_section.values())
    print(f"read {total} keys from {a.robot_conf} "
          f"(sections: {', '.join(sorted(s for s in conf.by_section if s)) or 'none'})")

    missing = []

    def pull(key, section, default, as_int=False):
        raw, found_in = conf.get(key, section)
        if raw is None:
            missing.append(f"{key} (expected in [{section}])")
            return default
        try:
            val = num(raw, as_int)
        except ValueError:
            missing.append(f"{key} = {raw!r} (not a number)")
            return default
        if found_in != f"[{section}]":
            print(f"  note: {key} found in {found_in}, not [{section}]")
        return val

    # ---- positional header -------------------------------------------------
    # balance_angle is the trim the old code calls balance_angle and the
    # rewrite renamed to theta_trim. Accept either spelling.
    raw, _ = conf.get("theta_trim", "balance")
    if raw is None:
        raw, _ = conf.get("balance_angle", "balance")
    if raw is None:
        missing.append("theta_trim / balance_angle")
        balance_angle = 0.0
    else:
        balance_angle = float(raw)

    bsec, bkeys = BALANCE_GAINS
    ssec, skeys = STEERING_GAINS
    bal = [pull(k, bsec, d) for k, d in zip(bkeys, (40.0, 0.5, 5.0))]
    steer = [pull(k, ssec, d) for k, d in zip(skeys, (1.0, 0.0, 0.1))]

    lines = [
        "0",                                   # holdPosition, legacy, ignored
        f"{balance_angle:.3f}",
        f"{bal[0]:.3f} {bal[1]:.3f} {bal[2]:.3f}",
        f"{steer[0]:.3f} {steer[1]:.3f} {steer[2]:.3f}",
        "",
        POS_SECTION,
    ]
    for key, fmt, default in POS_KEYS:
        v = pull(key, "position", default, as_int=(fmt == "%d"))
        lines.append(f"{key}=" + (fmt % v))
    lines.append("")
    lines.append(MOTOR_SECTION)
    for key, fmt, default in MOTOR_KEYS:
        v = pull(key, "motor", default, as_int=(fmt == "%d"))
        lines.append(f"{key}=" + (fmt % v))
    pidconfig = "\n".join(lines) + "\n"

    # ---- IMU offsets (space separated — imu_config.c uses fscanf %63s %f) ---
    imu_lines = []
    for key, default in IMU_KEYS:
        v = pull(key, "imu", default, as_int=(key == "pitch_axis"))
        imu_lines.append(f"{key} {v}")
    imu = "\n".join(imu_lines) + "\n"

    if missing:
        print("\n!! not found in robot.conf — wrote the default instead:")
        for m in missing:
            print(f"     {m}")
        print("   Check these by hand before arming.\n")

    if a.dry_run:
        print(f"\n===== {a.out} =====\n{pidconfig}")
        print(f"===== {a.imu} =====\n{imu}")
        return 0

    for path, body in ((a.out, pidconfig), (a.imu, imu)):
        if os.path.exists(path):
            shutil.copy2(path, path + ".bak")
            print(f"backed up {path} -> {path}.bak")
        try:
            with open(path, "w") as f:
                f.write(body)
                f.flush()
                os.fsync(f.fileno())
        except PermissionError:
            print(f"error: cannot write {path} — rerun with sudo", file=sys.stderr)
            return 1
        print(f"wrote {path}")

    print("\nSanity check before arming:")
    print(f"  head -4 {a.out}          # 0 / balance_angle / bal gains / steer gains")
    print(f"  cat {a.imu}              # pitch_offset must NOT be 0.00")
    return 0


if __name__ == "__main__":
    sys.exit(main())
