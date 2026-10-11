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

## 2026-10-07 — push and OOB recoveries fail the same way; three fixes

Captures: `bbot_ring_1791395611843` (67 s clean hold), `1791396254845` /
`1791396277660` / `1791396298590` (one push, overlapping ring dumps, merged
t=1169.6-1279.4), `1791397494105` (52 s clean hold, then a fall with no push).
Config throughout: kp 0.06 / ki 0.027 / kd 0.008, vel_scale_stop 3.7, mode 1,
qpps_max 800, max_amps 3.5, loop 98.7 Hz.

### The clean holds

| | pitch RMS | 1-4 Hz | duty RMS | enc p-p | drift |
|---|---|---|---|---|---|
| 1791395611843 | 0.83 | 13.2 | 0.051 | 36 | -2 / 67 s |
| 1791397494105 (first 52 s) | 0.75-1.1 | -- | 0.035-0.045 | ~20 | -- |

Both runs are one limit cycle at **0.75 Hz**, constant amplitude (per-cycle p-p
2.26 +/- 0.25 deg). The setpoint drives it, not the pitch loop: pit_setpoint RMS
0.90 vs measurement 0.83, 86% of the setpoint is velDamp. Neither damper is
clipping any more (velDamp max 1.78 of 2.0, posCorr max 0.32), so the 09-20
relay regime is gone. What is left is the actuator:

- commanded qpps vs RoboClaw-reported qpps: r = 0.11 at any lag to 0.6 s, sign
  agreement 49%. Commands of 0-10, 10-20, 20-40, 40-80, 80-200 qpps all
  produce the same 10-15 qpps actual. Balancing in place is 1 encoder count
  per 70-100 ms; the RoboClaw's internal speed estimate is noise at that rate
  and its velocity PID is regulating noise.
- pit_output -> enc_velocity lag **0.29 s** (same 280 ms as 09-20). 78 deg of
  phase at 0.75 Hz. That is the cycle.

Still the case for `mode = 0`. Not done in this entry; the three fixes below
apply in either mode.

### The push (1791396277660, t=1227.5-1229.5)

1. While pushed, setpoint sat at its cap (-3.54: velDamp 2.0 + posCorr 1.0)
   with duty ~0. Released at 1228.56 with 3.5 deg of stored lean: +5.4 deg at
   40 deg/s in 0.4 s.
2. The wheels had been dragged unevenly, so yaw_error reached **-48 deg**.
   yaw_output -0.31, and the mixer gave it equal say: 1229.05 L +0.07 / R -0.55,
   1229.10 L +0.38 / R -0.11 -- **wheels in opposite directions mid-fall**.
3. 1229.01-1229.10: L 3.52 A, R 3.70 A, over-current warning bits set
   (status 0x30000). 1229.16-1229.41: commanded -158 -> -785 qpps, left wheel
   pinned at -107..-117, **both motors -0.06..-0.15 A**. 250 ms of zero torque
   at full command while pitch rate went -50 -> -134 deg/s. Battery 10.8 V min.

### The OOB flip-flop (nine clears, 1229-1261)

Every clear is at +/-12 deg with the body swinging through at 30-76 deg/s.
Within 150 ms velDamp is at 2.0 and posCorr at 1.0 -- setpoint +/-3.1 before
upright. P and D cancel crossing upright (P -0.34, D +0.63 at 1241.31), net
0.25-0.40 duty against 76 deg/s. Out the far side, output 0.8, amps hit 3.5 A
(3.09, 3.51), same pinned-speed / ~0 A starvation. OOB the other way 0.5-1.2 s
later, identical every cycle.

### The no-push fall (1791397494105, t=2461.65-2462.58)

An ordinary larger swing of the 0.75 Hz cycle (pitch +2, encVel 7). Right wheel
49 qpps, left 24. 2462.01: **right motor 3.36 A**, over-current bit
(0xC0010000). Right speed +33 -> -23 -> -87 in 80 ms, left stays +15 -> 0: the
right wheel lost drive and was back-driven as the body pitched back. Then
yaw_error 2 -> 8 -> 37 -> 69 deg, yaw_output +0.60, right duty **pinned at
1.00**, left 0.0-0.28 of a 1.12 balance command. Spun (yaw_error at the 90
clamp), OOB 0.6 s after the trigger.

Every loss of balance in all five files starts with a motor at ~3.5 A.

### Changes (this build)

1. **Pitch D on the gyro.** `[pitch] d_src = 1` (default). Differenced DMP
   angle vs `state.theta_dot`: identical below 10 Hz (r=0.88, same sign), 134
   vs 28 in the 10-49 Hz band. Duty 10-49 Hz band was its largest band (1.90).
   Lets kd go to ~0.014 (what catching 70 deg/s needs) without the chatter.
   `d_src = 0` restores the old path.
2. **Yaw authority cap in the mixer.** `[yaw] max_authority = 0.15`,
   `taper_start = 0.5`. |yaw_output| <= max_authority, and the cap shrinks
   linearly to 0 as |balance_output| goes taper_start -> 1.0. A wheel at 1.0
   cannot take a differential; it would only subtract from the other wheel.
   Logged as `yaw_applied` and `yaw_lim`.
3. **Position loop held off after an OOB clear.** `[position]
   hold_after_oob_ms = 500`. Target tracks the wheels, correction and damping
   0, pos_last_correction stays 0 so max_angle_rate eases it back in.
   Logged as `pos_hold` (1 = hold, 2 = ramping back in over a second
   hold_after_oob_ms window; added 2026-10-08 after t=360.82 showed velDamp
   stepping 0 -> -2.0 in 150 ms when the hold lifted).
4. Ring header now records `yaw_mixer`, `pitch_d_src`, `hold_after_oob_ms`.
   Three new columns at the end of every row: `yaw_applied,yaw_lim,pos_hold`.

### Config, not code (robot.conf on the bot)

- `max_amps 3.5` -> the board's limit, or at least 10. Recovery is a 200 ms
  pulse. This is the most certain single change in this entry.
- After the gyro-D build is confirmed: kd 0.008 -> 0.012 -> 0.015, score the
  1-4 Hz band and duty RMS per run.
- `mode = 0` remains the fix for the 0.29 s actuator lag. Separate session;
  expect the first run to be worse before it is better.

UNTESTED on hardware. Bench first, wheels clear; then floor with a hand ready.

## 2026-10-08 (00:00) — first captures on the new build

`bbot_ring_1791432509685` (66 s hold), `1791432540143` (hold, two pushes, two
OOBs), `1791432362190` (mostly parked). 76 columns, header shows yaw_mixer and
pitch_d_src=1. qpps_max now 1000. **max_amps still 3.5.** Battery 10.9 V rest,
9.7 V min under load (min_batt_v is 9).

Hold, 66 s, vs the 10-07 afternoon runs:

| | 10-07 | 10-08 |
|---|---|---|
| pitch RMS | 0.75-1.1 | 0.50-0.78 |
| duty RMS | 0.035-0.045 | 0.025-0.037 |
| dTerm RMS | 0.043 | 0.020-0.031 |
| enc range | ~20-36 | ~14 |

Push #1 (t=344.6): pitch to 1.3 deg, recovered in 1.5 s, no OOB.

Push #2 (t=357.4-358.6): sustained push, setpoint at its 3.5 deg cap with ~0.1
duty for 0.5 s. On release: L 3.19/2.94/3.53 A, R 2.83 A at 358.17-358.29,
then 358.35-358.54 both motors -0.07..-0.13 A while commanded 752 -> 940 qpps,
wheels 151 -> 228. OOB at 358.60. Same max_amps signature as every 10-07 fall.

Yaw cap: confirmed working. yaw_lim went 0.15 -> 0.10 -> 0.03 -> 0 as the
balance command rose; the wheels got L -0.94 / R -0.89, no split.

Post-OOB hold: confirmed working. pos_hold=1 for 50 rows after the 360.28
clear; pitch came 11 -> 0 -> -4.7 deg and turned around (previous builds went
through to -18 and OOB'd the other way). Right motor hit 3.31/3.58 A at
360.67-360.77 and the wheels diverged while the hold was still on.

One defect found and fixed in this build: when the hold lifted at 360.82 the
velDamp term stepped 0 -> -2.0 in 150 ms (it is applied after max_angle_rate).
The position contribution is now ramped 0 -> 1 over a second hold_after_oob_ms
window; `pos_hold` logs 2 during the ramp.

Still not done: max_amps. Charge the battery.

## 2026-10-09 — best hold yet; push yield added

`bbot_ring_1791436262807`: mode 1, qpps_max 1500, max_amps 5, kd 0.008 on the
gyro. Battery 10.6 V at rest.

Hold, 50 s (t=874-925): pitch RMS **0.43-0.57**, p-p 1.0-1.8, duty RMS
0.026-0.038, enc held within ~10 ticks. Best capture on record.

t=929.8-930.6: pushed backward for ~1 s. velDamp pinned 2.0, posCorr 1.0,
setpoint at the -2.9 cap, duty ~0.05 -- the stored-lean case, exactly as
predicted on 10-08. On release (930.6+) pitch -3.5 -> -5.4, L 3.3/4.4 A, and
from 931.1 the wheel speeds stop matching the commands in sign (L commanded
-430..-806, measured +205..+756, 3.7-4.4 A). claw_vbatt sagged **10.5 -> 7.6 V**
during this (min_batt_v is 9). Whatever the RoboClaw does at 7.6 V main with
its PID loops saturated, it is not something to tune around: the pack is empty
or has gone high-resistance. 10.6 V resting on a 3S pack is ~15%.

Push yield added (this build): clean holds never saturate velDamp (671edc06:
0 ticks); pushes hold it at the cap for 60-135 ticks. `[position] yield_ms`
(300): velDamp at >= 97% of vel_damp_max for that long -> target follows the
wheels, spring 0, damping scaled by `yield_damp_gain` (0.25). Ends after
`yield_ms` below stopped_vel, then the hold ramps back in over
hold_after_oob_ms. `pos_hold` = 3 while yielding. Off with yield_ms = 0.

Expected on the next push capture: setpoint leaves the cap ~300 ms into the
push and the release starts from under 1 deg of lean instead of 3.

## 2026-10-09 (evening) — steering: two defects, one of them mine

Reported: dashboard D-pad does nothing for left/right; transmitter not doing
much either.

1. **The yaw nudge was overwritten.** `{"type":"nudge","axis":"yaw"}` did
   `state.yaw += d`, but robot.c does `state.yaw = yaw_target` every tick the
   stick path is live (transmitter on, or Pi applying). The nudge survived one
   tick. Now `robot_nudge_yaw()` moves `yaw_target` itself.
2. **The 10-07 yaw cap killed turning.** kp 0.005 x 15 deg nudge = 0.075 duty;
   x 45 deg (the IPC limit) = 0.225, clamped to 0.15 = 150 QPPS of
   differential at qpps_max 1000. The velocity loop delivers ~10-15 QPPS for
   anything under ~200 (10-07 measurement). Before the cap a 90 deg lead gave
   0.45 and it turned. Split the cap: `max_authority` (0.15) while holding
   heading, `max_authority_turn` (0.40) while a turn is commanded (stick/Pi x
   off centre, or a nudge in the last 2 s). Taper above taper_start applies
   to both. Added `turn_ff` (0.25): differential duty added straight from the
   stick/Pi x while held, so a turn starts without waiting for the heading
   error to grow. `yaw_applied` in the ring includes it.

Transmitter: not diagnosable from the captures so far -- every one was taken
with it off (in_drive, in_turn 0 throughout). Next: 20 s capture, TX on,
sticks moved. `in_drive`/`in_turn` show whether the sticks arrive at all;
`yaw_applied` and `cmd_wireL/R` show what the mixer did with them.

## 2026-10-10 (21:15) — BASELINE: mode 0, outer loop at half gain

`bbot_ring_1791681529337`, 250 s, armed and balancing throughout, no OOB, no
yield, target fixed. Whole run: pitch RMS **0.32 deg**, worst 5 s window
0.43, |pitch| never above 1.0. Position -14..35 ticks (2.5 in) over four
minutes. Duty RMS 0.04, current 0.35 A mean / 1.2 A peak. No spectral band
above 10 -- nothing cycling. Previous best (10-07, mode 1): 0.83 RMS, 36
ticks, 1.5 A, 67 s.

Configuration (config/overrides.conf, pushed with tools/push_conf.sh):

    [motor]       mode 0  qpps_max 1000  max_amps 5  friction_ff 0.05
    [pitch_duty]  kp 0.14  ki 0.035  kd 0.010   (d_src 1, gyro)
    [yaw_duty]    kp 0.02  ki 0  kd 0
    [position]    vel_src 0  vel_damp_fc 0  vel_scale_stop 7.4
                  scale_d 180  lead_max 120  (spring cap 0.67 deg)
                  yield_ms 300  yield_err_ticks 120  hold_after_oob_ms 500
    [system]      oob_angle_deg 12.5  reengage_deg 6  reengage_rate 30

How it got here, in one line each:
- mode 0 removed the RoboClaw velocity loop's 200-400 ms dead time
  (cmd -> wheel lag 20-40 ticks -> 1 tick).
- duty gains: kd 0.03 rang at 3.9 Hz; 0.010 does not. kp 0.14.
- friction_ff 0.05: wheels answer 0.02 duty (34 qpps) instead of stalling.
- the 0.8 Hz hunt was the velocity-damping loop (velDamp pinned at its
  +/-2 cap every half cycle) through the body's 0.2 s tracking lag;
  vel_scale_stop 3.7 -> 7.4 with scale_d 90 -> 180 and lead_max 60 -> 120
  halved the loop gain at the same spring/damper ratio and spring cap.
  vel_src 1 and vel_damp_fc 3 were tried and reverted: a switch to vel_src 1
  at 6089 preceded a fall within 3.6 s.
- yield_err_ticks 120: the yield was re-homing the bot on its own swing.

Open: a motor channel (left 10-10, right 10-07/10-09) drops to ~0.1 A under
large commands, intermittently. Not seen in this run (peak 1.2 A).

Next: one push at this damping; then scale_d 150 / lead_max 100 for a firmer
return to spot.

## 2026-10-10 ~22:00  root cause of the "dead motor channel": regen clamp

The "Open" item above is resolved. The RoboClaw main-battery max voltage
(cmd 57, dashboard Batt Max) read back 0. With max = 0 the bus is always
above the limit, so the controller clamps regenerative braking: whenever
the loop commands a large duty against wheels that are rolling the other
way (every push recovery, every post-OOB catch), the commanded channel
delivers ~0 A / slightly negative current and the wheel freewheels. That
is the -0.1 A "dead channel" seen on the left 10-10 and the right
10-07/10-09, and the reason the falls happened at -1.0 duty with no
braking. It was never wiring.

Why the firmware did not fix it: motor_hal_set_batt_limits with
max_batt_v = 0 reads the controller's 0, fails validation (ret -100)
and leaves the controller as it was.

Fix: Batt Max set to 16 V on the dashboard; [motor] min_batt_v = 9 /
max_batt_v = 16 pinned in config/overrides.conf so a push_conf restores
it. Verification pending: repeat the hard push and expect 3-5 A braking
current at the -1.0 duty moment with the wheels decelerating within a
couple of ticks.

Verification result (bbot_ring_1791682959092, push at 27.9 s): NOT fixed.
With Batt Max reading 16 V, the braking quadrant still delivers nothing:
duties -1.0 against wheels at +476/+385 qpps, currents 1.75/0.14 A then
-0.13/-0.10 A within 80 ms, wheel decel 360 qpps/s under full reverse
command versus 635 qpps/s coasting after the OOB cut. Over the whole
capture, ticks with |duty|>0.3 and |speed|>100: motoring quadrant means
3.2 A (L) / 2.7 A (R); braking quadrant 0.36 A (L) / 0.20 A (R). Both
channels, same shape. Main-voltage-high warning bit never set, but vbatt
is sampled at ~10 Hz and a regen spike into a bus that cannot sink current
would be clamped by the controller in well under 1 ms.
So the threshold was not the limiting factor. Open question is the power
path: whether anything between the RoboClaw main terminals and the cells
(diode, ideal-diode, BMS discharge FET, the Blue's DC-in path) blocks
reverse current. If regen has nowhere to go the RoboClaw clamps it
regardless of max_batt_v.
