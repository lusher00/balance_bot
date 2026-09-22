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

## 2026-09-19 (evening) — kicks, recovery, pushes

Plant/firmware changed under this log: motor mode 1, qpps 500; pitch
derivative now on measurement (see below). Earlier rows above predate both.

**Best calm hold, reproduced twice (looplogs 12, 16, 20 s each):**
pitch 0.06/0.027/0.008, `scale_d 55`, `vel_scale_stop 4.5`, `vel_damp_max 2`,
`pos_ki 0.005 pos_i_max 0.5`, `max_angle_rate 0.15`, `max_correction 6`.
Pitch RMS 0.35–0.37, encErr sd 5.5–8.5, drift 2–4 ticks. `vss 6` doubled the
0.15–0.6 Hz hunting; `vss 4` rang (1–4 Hz band up, duty RMS 2×, one fall).

**Derivative kick (fixed in code):** d_term was on error, and the setpoint
(position loop, RoboClaw speed updating every ~5 ticks) steps. Setpoint part of
d_term sd 0.068–0.076 vs body motion 0.043–0.055; single-tick kicks to 0.81
duty. After switching to derivative-on-measurement: d_term sd 0.034, jumps
gone. Same kd, same damping on real motion.

**`vel_damp_max` 2 → 6: worse.** On recovery the pitch loop's catch spins the
wheels; the brake reads that as motion and commands ~7 deg the other way while
the body swings through upright → over to −18. Keep 2.

**Pushes:** the failure is always the lean command. Commanded lean stays within
~±4 deg → body follows and holds. Error > ~70–100 ticks → spring + brake push
it to 5.5–6.5 deg (`max_correction` 6) → body follows it over. Two responses
queued: `max_correction 3` alone, and the return-home carrot
(`lead_max 40`, `return_rate 100`).

**Restoring the Sep 2 set** (0.07/0.02/0.005, vss 5, rate 0.08, trim from an
older zero) bounced and fell: kp 0.07 rings on this plant (Sep 13 already
showed it) and the trim was 2.1 deg off, holding a ~90-tick standing error.

## Next

1. `max_correction 3`, same pushes — does it arrest without falling?
2. Carrot on (`lead_max 40`, `return_rate 100`), `max_correction` back to 6 —
   compare.
3. Only then revisit `scale_d` / `vel_scale_stop` for the slow sway
   (0.15–0.6 Hz band 3.2 on the 219 s run vs 0.8–1.2 in runs 12/16).

---

## 2026-09-20 — yaw jitter is encoder quantisation, not noise

Standing question finally answered: the idle yaw dither, present even on the
kickstand, is a limit cycle driven by the resolution of the heading signal.

`phi_diff = (phi_R - phi_L)/2` moves only in whole encoder counts of
`360 / 145.1 / 2 = 1.2405 deg`. Every distinct `yaw_measurement` value in
`bbot_ring_1789927479098.csv` (79 of them) is an exact multiple of that. The
D term was differentiating a staircase: at `kd 0.0005`, 100 Hz, one count is
`0.0005 * 1.2405 / 0.01 = 0.060` of duty differential, against `kp * 1.2405 =
0.0062` from the proportional term. **10:1, derivative over proportional, from
a signal carrying no rate information.**

65 s parked and upright, heading wandering +/-5 counts:

| metric | value |
|---|---|
| yaw d_term rms | 0.0338 |
| yaw p_term rms | 0.0081 |
| yaw output rms | 0.0363 |
| output sign reversals | 25 /s (~12 Hz) |
| mot_dutyDiff rms | 7% |
| wheel travel | 1786 counts to net 5 |

One count -> D impulse -> duty differential -> free wheels turn -> next count.
Worse unloaded, which is why the kickstand shows it most clearly.

**Change:** `yaw_gyro_scale` (`[yaw] gyro_scale`). 0 = the old encoder
derivative, still the default. Nonzero switches the D term to gyro Z
(`state.psi_dot`), scaled into phi_diff deg/s with the sign folded in.
`pid_update_rate()` added alongside `pid_update()`. The ring now logs
`yaw_gyroRate` and `yaw_encRate` so the scale is fitted from a run rather than
derived from a tape measure — see README "Yaw Derivative Source".

Also this session: the carrot's cruise-speed clamp (added earlier the same day,
capping speed at 80% of brake authority) was the wrong lever and made every
move crawl at ~72 ticks/s regardless of `return_rate` — that is the "way too
slow" with `return_rate 400`. Replaced with an acceleration clamp: a lean of
t degrees buys ~135 ticks/s^2 of decel, the brake is capped at `vel_damp_max`
degrees, spend 70% of it. Speed is free; the existing `v <= sqrt(2 a d)`
ramp-down is what guarantees any cruise speed still stops in the distance left.

### Next
1. Kickstand, `gyro_scale = 1`, turn by hand ~20 s, dump the ring, fit
   `yaw_encRate` vs `yaw_gyroRate` for sign and magnitude.
2. Enter the fitted scale; idle `mot_dutyDiff` should fall from ~7% rms to
   near zero and the sign reversals should stop.
3. Then re-test move commands with the accel clamp, and only after that
   revisit `scale_d` / `vel_scale_stop` for the slow sway.

## 2026-09-20 (later) — moves diverge: the carrot was following the bot

`bbot_ring_1789928811195.csv`. Holds position fine (the no-move capture
`..773114` is 100% upright, |pitch| max 1.9). Every move turns into a growing
oscillation, ~0.22 Hz, overshoot **176 -> 211 -> 440 ticks** over three
half-cycles, then it goes over.

It is not a failed arrest. It is a limit cycle, and the loop is open for most
of it. Over the 10.5 s of the move:

| | |
|---|---|
| `pos_posCorr` pinned at +/-1.000 | 70% of ticks |
| `pos_velDamp` pinned at +/-2.000 | 75% of ticks |
| both pinned at once | **57% of ticks** |
| motor duty | rms 0.247, max 0.869 — **never saturated** |

`lead_max / scale_d = 40 / 40 = exactly 1.0`, so the spring hits its ceiling
the moment the carrot reaches full lead and carries no distance information
after that. With the damper also pinned at `vel_damp_max`, the output is a
constant +/-3.5 deg square wave. There is nothing left to regulate with, and
the wheels had authority to spare the whole time.

Three causes, all fixed in `robot.c`:

**1. The lead clamp rewrote the plan.** `carrot` was clamped to
`here +/- lead` and written back, so once the bot overtook it, the carrot was
dragged along by the runaway and the trapezoid re-planned a fresh full-speed
move from wherever it landed. `pos_carrotVel` flipping +89.7 -> -11.3 -> -30.2
mid-move is the plan being rewritten. Now the carrot keeps its own trajectory,
latches on the target when it arrives (`carrot_arrived`), and only the ERROR
is clamped to `lead_max`. An overshooting bot gets a steady pull home instead
of a new acceleration command.

**2. Position integrator anti-windup never fired.** It tested
`|pos_last_correction| >= max_correction * 0.98` — 5.88 deg — but with the
carrot on, the spring cannot exceed `lead_max / scale_d = 1.0`. So it wound
freely at `pos_ki * err = 0.005 * 40 = 0.2 deg/s` to its 0.5 cap (visible as
the 0.020-per-100 ms creep in `pos_thetaAdj`) and sat there, 0.5 deg of stale
lean that every reversal had to unwind first. Now it also freezes on
`pos_lead_saturated`, and only against the limit — unwinding is still allowed.

**3. Damping feed-forward.** `carrot_v_act` stopped meaning anything once the
carrot kept its own plan; `v_ref` now comes from `enc_velocity` directly,
capped at the planned speed and zeroed unless the signs agree. Same three
protections as before (blocked -> no phantom lean, pushed -> full damping,
cruising -> no brake), read off the bot instead of off the carrot.

Checked and NOT the cause: the wheels were never at the stops (max duty 0.87),
and `enc_velocity` does not track `pit_output * qpps_max * 2/10` (r = 0.29),
so the velocity-mode inner loop is not slaving travel speed to pitch output.

### Next
1. `vel_damp_max 2 -> 4`. It is pinned 75% of the move; that is the brake
   ceiling, and `max_correction 6` leaves room for it. It also lifts the accel
   clamp from 189 to 378 ticks/s^2, so moves get quicker at the same time.
2. Re-run the same move commands. `pos_velDamp` should stop living on the cap
   and the overshoot should decay instead of grow.
3. Only then go back to the yaw gyro-D calibration.

**Regression in that change, same evening:** `carrot_arrived` latched and
nothing cleared it. Standing still counts as arrived (`dist == 0` -> `dir == 0`
-> arrived), and a move command only assigns `state.enc_pos_target` -- it never
reaches into the plan. So from the first tick after arming, every move ran with
`carrot_v = 0`: no trapezoid, no feed-forward, and the damper opposing the
entire speed instead of just the excess over plan. Equilibrium is where a 1 deg
spring balances `vel_scale_stop`, about 33 ticks/s / 42 mm/s -- indistinguishable
from not moving. The plan now watches `enc_pos_target` itself
(`carrot_home_prev`) and re-plans from the carrot's current position and speed
whenever it changes, so a move commanded mid-move bends the trajectory instead
of restarting it.

Tell for this one in a log: `pos_carrotVel` sitting at 0 while `pos_encError`
is large.

## 2026-09-20 (later still) — the trim box was measuring the wrong signal

It never asked for trim because it read `pos_thetaAdj`, the POSITION loop's
output, and the estimator only accepts samples taken **parked at the target** --
the one condition under which the position loop has nothing to correct and its
output goes to ~0 regardless of trim. The offset lands in the PITCH loop
instead, as a standing error.

`bbot_ring_1789928773114.csv`, 1448 parked-and-quiet ticks (|err| <= 25,
|encVel| <= 2):

| | |
|---|---|
| median `pos_thetaAdj` (what it read) | **+0.19** -> "trimmed" |
| median `pit_setpoint` | +0.19 |
| median `pit_measurement` | **-0.71** |
| actual standing error | **0.90 deg** |

Now reads `pitch.measurement - pitch.setpoint`. `telemetry.c` sets
`pitch.setpoint = theta_ref + theta_offset`, so that difference is both the
error and the correction: `theta_offset += (measurement - setpoint)` drives it
to zero, and `biasApply()` keeps applying half of it.

**Why the standing error does not integrate away:** `pitch_pid.integrator_max`
is the `pid_init` default of 1.0 deg-s, so the i_term cannot exceed
`ki * 1.0 = 0.027` duty, which at `kp 0.06` cancels only **0.45 deg** of
standing offset. The measured offset is 0.90 -- double the integrator's whole
authority, so it simply sits there. (`robot.c:594` has a commented-out
`integrator_max = 4.0f`.) Raise the trim, not the limit: a bigger integrator
would mask the offset rather than remove it, and it is the same offset that
makes the drivetrain gain asymmetric (87.7 vs 57.2 enc_velocity per unit
pit_output, 2026-09-20 `..931001783`).

## 2026-09-20 (evening) — RETRACTION: no drivetrain asymmetry

The "1.5x directional gain asymmetry" reported earlier was a **fitting
artifact**. It came from through-origin regressions of `enc_velocity` on
`pit_output` computed separately for positive and negative output; the two
halves have different output-magnitude distributions, so the slopes differ
without the underlying curve differing.

Binned non-parametric check, `bbot_ring_1789931712395.csv`, 0.1-duty bins,
median achieved `enc_velocity` at a 280 ms lag:

| pit_output | achieved | ideal (x100) |
|---|---|---|
| -0.5 | -38.2 | -50 |
| -0.4 | -35.5 | -40 |
| -0.3 | -20.1 | -30 |
| -0.2 | -18.4 | -20 |
| -0.1 |  -6.8 | -10 |
| +0.1 |  +3.9 | +10 |
| +0.2 | +21.3 | +20 |
| +0.3 | +32.0 | +30 |
| +0.4 | +38.7 | +40 |
| +0.5 | +46.1 | +50 |

Symmetric, and within ~20% of ideal except near zero where stiction bites.
The tell I should have caught: the ratio was **1.53 before the trim change and
1.53 after** (87.7/57.2 -> 78.2/51.2), while the standing lean moved from
+0.89 to -0.15 deg. A lean-caused asymmetry would have moved with it.

**The trim fix itself worked.** `theta_trim` 0 -> 0.145, and:

| | before | after |
|---|---|---|
| mean pitch while stationary | +0.89 | **-0.15** |
| standing error (meas - setpoint) | 0.90 | **0.37** |
| stuck (still, \|err\| > 40) | 29% | **9%** |

## What is actually wrong: a saturating relay loop

Steady stretches (no target change, upright), same log:

| | |
|---|---|
| `pos_velDamp` pinned at +/-2.0 | **33%** of ticks |
| `pos_posCorr` pinned at +/-1.0 | **49%** of ticks |
| velocity sign flips | 45, median half-period **0.88 s** (~0.57 Hz) |
| `encError` | median 48, p90 271, max 614 -- never settles |

Actuation lag `pit_output` -> `enc_velocity` is **280 ms** (r = 0.81). A loop
with that much delay and a *saturating* damper is a relay oscillator: once the
damper clips, the loop is bang-bang and bang-bang with delay can only limit
cycle. That is the same defect as the growing move oscillation, and it is
independent of the carrot.

### Next — reduce gain, do not add authority
1. `vel_scale_stop` 3.3 -> **8**. In the cycle `encVel` reaches ~8, giving
   `8/3.3 = 2.4` against the 2.0 cap. At 8 it stops clipping at the speeds it
   actually sees, which takes the loop out of the relay regime.
2. `scale_d` 40 -> **80**. With `lead_max 40` the spring saturates at exactly
   `40/40 = 1.0` and does so 49% of the time; at 80 it stays proportional.
3. Both are gain REDUCTIONS. It will feel slower. Settling first, speed after.

## 2026-09-20 (late) — yaw gyro-D made calibratable from any run

The gyro-D path was built but could not be calibrated without a deliberate
setup step: `yaw_gyroRate` logs `psi_dot * yaw_gyro_scale`, and the scale is 0
until it is calibrated, so the column was all zeros in exactly the situation
where it was needed. Confirmed on `bbot_ring_1789948917626.csv`: `yaw_gyroRate`
nonzero on 0 of 5978 rows while `yaw_encRate` was live on 2313.

The ring now also logs **`yaw_psiDot`** -- raw chassis yaw rate from gyro Z,
deg/s, unscaled and therefore always populated. `tools/yaw_gyro_scale.py`
prefers it and falls back to the old scaled column (dividing the live scale
back out) for older captures, with an explicit message when that path has
nothing to fit. No special run, no temporary scale of 1: turn the robot by hand
for ~20 s during any capture and fit it.

Motion gate checked and confirmed wired (`robot.c:728` installs
`robot_motion_gate` via `motor_hal_set_motion_gate`); it was not inert.

Boot takeoff considered diagnosed and fixed by Ryan.

## 2026-09-20 (late) — REGRESSION: the anti-windup made it park short

`bbot_ring_1789950092231.csv`. Same config as the good run, but the robot
repeatedly stops 100-190 mm short of the target and sits there.

Cause: my conditional-integration change also froze `pos_integ` on
`pos_lead_saturated` (the carrot error at `lead_max`). That is exactly backwards
-- the spring being at ITS cap is precisely when the integral is supposed to
supply the authority the proportional term has run out of.

Measured, t = 50.5-53.7 s, stopped 150 ticks from target:

| | |
|---|---|
| `pos_encError` | -135 to -161, not closing |
| `pos_posCorr` | pinned at -0.889 |
| `pos_posIterm` | **frozen at +0.325 for 3.2 s** |
| `pos_thetaAdj` | 3.21, against `max_correction` 6 |
| unused output headroom | **2.8 deg** |

16% of the whole 66 s file is spent parked >60 ticks off target with the
wheels stationary, median error 79 ticks (100 mm).

Fixed: the freeze now tests the OUTPUT clamp only (`|pos_last_correction| >=
max_correction * 0.98`), which is textbook conditional integration.
`pos_lead_saturated` removed entirely.

### Measurement note for future runs
This capture opens with the robot already down at -18 deg -- the fall happened
before the ring window and was not recorded. Naive whole-file stats made it
look far worse than it was (pitch SD 18.8 including 14 s of lying on the floor;
3.68 over the 39.6 s it was actually balancing). **Dump the ring immediately
after a fall**, and segment on `|pitch| < 12` before computing anything.

## 2026-09-20 (late) — a twist never recovers: the heading target was being re-adopted

Not tuning. `robot.c` reset the steering target to the current heading on
every tick:

```c
if (!state.armed || !sbus_is_connected())   /* <- the second half */
{
    yaw_target = (state.phi_right - state.phi_left) / 2.0f;
    state.yaw  = yaw_target;
}
```

**This robot is normally driven with the transmitter OFF**, so
`!sbus_is_connected()` was true on every tick and the heading target was
reassigned to wherever the robot happened to be pointing, 100 times a second.
A twist became the new home before the loop could correct it.

Measured, `bbot_ring_1789950092231.csv`, 5013 upright ticks:

| | |
|---|---|
| median \|yaw_setpoint - phi_diff\| | **0.000** |
| median \|yaw_error\| | **0.000** |
| max \|yaw_error\| | 90.000 -- exactly `MAX_YAW_LEAD` |

The loop was regulating against a target that *was* the measurement.

Three fixes:

1. The `!sbus_is_connected()` clause is gone; only `!state.armed` forgets the
   heading. Losing the link is a reason to stop accepting new stick commands,
   which the drive path already handles -- not a reason to abandon the heading
   the robot is standing on.
2. `MAX_YAW_LEAD` now clamps the **error the PID sees**, not `state.yaw`. It
   used to write the clamped value back into the target, so a twist larger
   than 90 deg dragged the target along and the original heading was gone for
   good -- the same defect as the position carrot's lead clamp, fixed the same
   way. The anti-windup bound is preserved; the target survives.
3. `telemetry.c:174` computed `yaw.measurement` as `(phi_left - phi_right)/2`
   while the loop closes on `(phi_right - phi_left)/2`. Every log and dashboard
   reading had `yaw_measurement` **negated** relative to `yaw_setpoint` and
   `yaw_error`, so `yaw_error != setpoint - measurement` and the columns read
   at face value gave the wrong turn direction. Now consistent.

Note that (3) means yaw columns in captures before this build have the opposite
sign to ones after it.

## 2026-09-20 (late) — stand-up kick

Parked on the kickstand at 17-20 deg the robot is outside `oob_angle_deg`
(12.5), so every actuation path is dead and it cannot be stood up without
being picked up. Added a bounded, operator-held kick: drive stick past 25%, or
hold the dashboard's KICK buttons. Both go through one function,
`kick_direction()` in robot.c, so the guards cannot be bypassed by either path.

Window is `oob_angle_deg` to `kick_max_deg` (25). Guards: armed + IMU fresh,
release-before-each-kick latch, `kick_timeout_ms` (1200) per kick, a 300 ms
staleness deadman on the dashboard request, and it still goes through
`motor_hal_set_both()` so the motion gate applies. `kick_assist = 0` disables
it. New `[system]` keys: `kick_assist`, `kick_max_deg`, `kick_duty` (0.35),
`kick_timeout_ms`.

UNTESTED on hardware. First run should be on the bench with the wheels clear.
