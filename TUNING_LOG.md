# Position hold — tuning log

Every run, appended. Pairwise comparisons go stale; the accumulated map doesn't.

Fixed across all of these unless noted: `zone_a=8000 zone_b=4000 zone_c=500`,
`scale_a=60 scale_b=80 scale_c=200`, `vel_scale_move=70 vel_scale_turning=70`,
`stopped_vel=5 max_correction=6 max_angle_rate=0.1 back_to_spot=1`,
pitch gains `kp=0.07 ki=0.02 kd=0.005`.

Only `scale_d` and `vel_scale_stop` have moved.

| scale_d | vel_scale_stop | dur | \|encErr\| | encErr sd | pitch sd | %\|pe\|>3 | %<20 | verdict |
|---|---|---|---|---|---|---|---|---|
| 50 | 25  | 27 s  | 202.9 | 205.7 | 7.52 | 34.20 | 6.9  | **UNSTABLE** — fell over |
| 35 | 3.5 | —     | —     | —     | —    | —     | —    | **UNSTABLE** |
| 50 | 5   | 62 s  | 28.7  | 24.9  | 2.00 | 2.72  | 38.5 | stable, sluggish |
| 40 | 5   | 320 s | 22.6  | 17.6  | 1.69 | 1.42  | 46.1 | stable — **calmest** |
| 40 | 5   | 108 s | 21.3  | 16.4  | 1.50 | 0.60  | 48.6 | *(first 108 s, outlier window removed — the fair slice)* |
| 35 | 5.5 | 108 s | 18.6  | 19.3  | 1.93 | 0.37  | 54.4 | stable — **best centering** |

Lower is better for `|encErr|`, `encErr sd`, `pitch sd`, `%|pe|>3`.
Higher is better for `%<20` (fraction of samples within 20 ticks of target).

## What the map says so far

**Centering improves monotonically as `scale_d` drops** (50 → 40 → 35 gives
28.7 → 21.3 → 18.6 mean error). No sign of a floor yet.

**Calmness does not.** Pitch sd goes 2.00 → 1.50 → 1.93 — 40/5 is the quietest
point found. Whether the rise at 35 came from the gain or from the damping
moving 5 → 5.5 is unresolved.

**Stability is bounded on both sides in `vel_scale_stop`.** 3.5 is unstable
(too much damping), 25 is unstable (too little), 5–5.5 works. The staircase
explanation for the upper bound was tested and disproved — see the rejected
section below. `POS_VEL_PERIOD_MS` is 40, not 100, so it was a 25 Hz update
rather than 10 Hz, and the RoboClaw filters internally so it was never steppy.
The surviving explanation is the RoboClaw's measured 50–100 ms of lag: high
damping gain against that much phase lag is enough on its own to ring.

## Velocity estimator — tried and REJECTED (2026-08-22)

Hypothesis: the ~0.55 Hz ring persists because damping cannot be turned up, and
damping cannot be turned up because `enc_velocity` comes from the RoboClaw's
speed registers every 40 ms and lands as a staircase. Proposed fix: derive
velocity at 100 Hz from a least-squares slope of `enc_pos`.

**Both halves of that were wrong.**

1. The staircase theory is false. Measured head to head on the robot, the
   RoboClaw reading is **1.59x SMOOTHER** than a 6-tick slope — it filters
   internally, so holding its value between updates is gentle, not steppy.
   Switching the control path to the 6-tick estimate blind made the bot
   unbalanceable.
2. A longer window does not rescue it. Three windows logged simultaneously
   against the incumbent:

   | estimate | jerk vs RoboClaw | lead | predicted |
   |---|---|---|---|
   | LSQ 6  | 1.59x | 50 ms | 1.79x |
   | LSQ 12 | 1.16x | 50 ms | 0.69x |
   | LSQ 20 | 1.04x |  0 ms | 0.54x |

   No window wins. w=20 gives up all its lag advantage and is still jerkier.
   Every LSQ variant also has higher sd (6.0-6.3 vs 5.3) — they track motion
   the RoboClaw filters out, which is exactly wrong for a damping term.

Why the predictions failed: both were made against a simulation whose only
noise source was encoder quantisation. Real encoder noise is broadband, and a
longer window suppresses it far less effectively than it suppresses
quantisation. **The lesson is not "simulate better" — it is that the incumbent
was never characterised before being replaced.**

What IS established and still true: the RoboClaw reading lags the position
signal by 50-100 ms. That lag is real and is the likelier reason
`vel_scale_stop = 3.5` rang. It just cannot be recovered this way.

The code keeps all three candidates behind `POS_VEL_USE_LSQ` (0 = RoboClaw
controls, the default and the only setting that balances). Strip the extra
candidates whenever the scaffolding stops being useful.

## Standing issues, independent of tuning

- **The offset is one-sided.** `encError` is ~88% positive against a fixed
  target and survives 5 minutes. That is a standing bias (centre of mass, or
  IMU zero), not wander. More gain shrinks it; it will not remove it. That is a
  `theta_offset` trim question.
- **Zones are inert.** `zone_c = 500` against errors that never exceed ~95
  ticks means `scale_d` is the only scale ever selected — `scale_a/b/c` are
  dead config.
- **`drive_mode`: bot=1, ui=0** on the last three runs. The dashboard has been
  wrong about the drive mode the whole time.

## Unexplored

- `35 / 5` — separates the last change into its two parts
- `30 / 5` — does centering keep improving, and what does it cost
- anywhere below `vel_scale_stop = 4` or above `6` — the stability edges are
  only known to lie somewhere between 3.5 and 25
