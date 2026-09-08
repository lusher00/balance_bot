// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
//
// This file is part of balance_bot, licensed under the PolyForm
// Noncommercial License 1.0.0. You may use, study, modify, and share
// it for any noncommercial purpose. Commercial use requires a separate
// license from the author -- contact ryan.lush@gmail.com.
// Full license text: see the LICENSE file in the project root, or
// https://polyformproject.org/licenses/noncommercial/1.0.0/

/**
 * @file balance_bot.h
 * @brief Main header for balance_bot
 *
 * Primary header — includes all module interfaces for the
 * balance_bot self-balancing robot with iPhone app integration.
 *
 * Architecture:
 * - balance:  Angle controller — keeps robot upright
 * - position:    Position controller — drives via lean angle (optional)
 * - yaw:      Yaw controller — turns left/right
 * - uart_input:    Generic packet-based UART input (external coprocessor, etc.)
 * - roboclaw:      Packet-serial motor driver over a dedicated UART
 * - ipc_server:    Unix-socket bridge to Node.js / iPhone app
 * - telemetry:     Real-time state broadcast to iPhone
 */

#ifndef BALANCE_BOT_H
#define BALANCE_BOT_H

#include "rc_compat.h"
#include <stdbool.h>
#include <stdint.h>
#include "debug_config.h"
#include "motor_hal.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define SAMPLE_RATE_HZ 100
#define DT 0.01f

// Control limits
#define MAX_THETA_REF 17.0f // Max lean angle command (deg)
#define MAX_YAW_CMD 1.0f   // Legacy: normalised steering, still used by MODE_EXT_INPUT
// How far the steering heading target may lead the actual heading, in degrees
// of phi_diff. Anti-windup: without it, holding the turn stick while the wheels
// are blocked (or the bot is lifted) winds the target up without limit and the
// bot spins hard the instant it regains traction.
#define MAX_YAW_LEAD 90.0f

// PID default gains (tunable via iPhone app)
#define PITCH_KP 0.050f
#define PITCH_KI 0.015f
#define PITCH_KD 0.005f
#define YAW_KP 0.010f
#define YAW_KI 0.000f
#define YAW_KD 0.002f

#define DRIVE_PHI_DEADZONE 2.0f

// position hold controller — encoder-tick-based hold/drive
// These defaults initialise g_pos_config in robot.c.
// Use set_pos_config IPC command or the iPhone app to tune at runtime.
#define POS_ZONE_A_DEFAULT 8000
#define POS_ZONE_B_DEFAULT 4000
#define POS_ZONE_C_DEFAULT 1000
#define POS_SCALE_A_DEFAULT 600.0f
#define POS_SCALE_B_DEFAULT 800.0f
#define POS_SCALE_C_DEFAULT 1000.0f
#define POS_SCALE_D_DEFAULT 500.0f
#define POS_VEL_SCALE_STOP_DEFAULT 60.0f
#define POS_VEL_SCALE_MOVE_DEFAULT 70.0f
#define POS_VEL_SCALE_TURNING_DEFAULT 70.0f // Turning authority reduction at speed
#define POS_STOPPED_VEL_DEFAULT 40
#define POS_MAX_CORRECTION_DEFAULT 10.0f
#define POS_MAX_ANGLE_RATE_DEFAULT 0.05f // deg/tick — ramps 0.5°/s at 10 Hz vel window
/* Drive stick interpretation. Defaults to the legacy lean mode so behaviour is
 * unchanged until you deliberately switch it. drive_rate 300 ticks/s is about
 * 1.0 m/s at full stick; runaway_limit 300 ticks is about 1.0 m of lead, well
 * inside zone_c so the loose-hold abandon branch never fires because of it. */
#define POS_DRIVE_MODE_DEFAULT 0
#define POS_DRIVE_RATE_DEFAULT 300.0f
#define POS_RUNAWAY_LIMIT_DEFAULT 300
#define POS_BACK_TO_SPOT_DEFAULT 1       // Full zone hold by default
#define POS_KI_DEFAULT     0.0f          // off — opt in from the dashboard
#define POS_I_MAX_DEFAULT  2.0f          // deg; ~4x the standing lean seen so far
#define POS_VEL_DAMP_MAX_DEFAULT 0.0f    // 0 = unlimited, original behaviour
#define POS_VEL_DAMP_FC_DEFAULT 0.0f     // off — opt in from the dashboard
#define POS_VEL_SRC_DEFAULT 0            // RoboClaw speed registers — known good
#define POS_DEADBAND_DEFAULT 2           // ticks; 2 = 2.6 mm = 0.1 in
// How often the RoboClaw is polled for wheel speed (ms).
//
// This is a POLL RATE, not a measurement window: roboclaw_encoder_speeds()
// returns an instantaneous hardware QPPS reading, and the /10 applied in
// robot.c is a pure unit conversion (pulses/sec -> ticks/100ms). So changing
// this does NOT rescale enc_velocity and does NOT invalidate vel_scale_stop.
//
// Was 100, which held one sample for 10 control ticks. Measured against
// d(enc_pos)/dt, reported velocity lagged true velocity by ~102 ms — 19 deg of
// phase error on the position hold damping term at the observed ~1.9 s position limit
// cycle. Damping that far out of antiphase stops opposing motion and starts
// behaving like a position term, producing a ~100-150 mm rocking oscillation
// that no value of vel_scale_stop or scale_d could remove.
//
// One serial round-trip per poll. If loop_hz drops below ~100, raise this.
#define POS_VEL_PERIOD_MS 40

/* Encoder velocity: least-squares slope of enc_pos over a sliding window,
 * computed every control tick. Three windows are evaluated simultaneously so a
 * single run picks the winner from data instead of from a simulation -- the
 * first attempt at this was chosen by simulation, and the simulation modelled
 * the incumbent as far worse than it is. See the comment in robot.c.
 *
 * Approximate lag is (win-1)/2 control ticks: 25 ms, 55 ms, 95 ms.
 * The RoboClaw incumbent measured 50-100 ms BEHIND the 6-tick estimate. */
#define POS_VEL_WIN_SHORT 6
#define POS_VEL_WIN_MID   12
#define POS_VEL_WIN_LONG  20
#define POS_VEL_WIN_MAX   20    /* must be >= the largest of the three */

/* 0 = RoboClaw speed registers control the loop (known good, default)
 * 1 = short window controls, 2 = mid, 3 = long
 * All candidates are computed and logged regardless.
 *
 * Sep 2 2026 -- set to 2 (mid window).  The RoboClaw's own speed registers are
 * polled every POS_VEL_PERIOD_MS and held, and measured against the true
 * derivative of enc_pos they lead position by only +76.9 deg at the 0.33 Hz
 * wander frequency where +90 is ideal.  The mid window gives +86.0.
 *
 * That 13 deg matters more than it looks, because velocity damping closes an
 * inner loop (velocity -> lean -> acceleration -> velocity) whose stability is
 * set by the phase its velocity signal carries.  With a late signal there is no
 * good gain: vel_scale_stop=7 is underdamped, 5 is the optimum, and 3.5 sends
 * the damping term up 8.7x for a 1.43x gain increase and pins the lean command
 * at its clamp.  Gain cannot fix phase.
 *
 * IMPORTANT -- the two sources are NOT the same scale.  Measured over a 95 s
 * run, enc_vel_mid = 1.255 x vel_claw (r = 0.934).  Since vel_damp is
 * enc_velocity / vel_scale_stop, changing this constant without also scaling
 * vel_scale_stop raises the damping gain by 26% as a side effect -- on a loop
 * where 43% was catastrophic.  vel_scale_stop must go 5.0 -> 6.3 in robot.conf
 * at the same time, which holds the gain constant and leaves phase as the only
 * thing that changed. */
#define POS_VEL_USE_LSQ 0   /* compile-time DEFAULT only; pos_config.vel_src
                             * overrides it at runtime. Left at 0 so a board
                             * with no robot.conf comes up on the known-good
                             * source. */

/**
 * @brief Runtime-tunable parameters for the position hold (hold/drive) controller.
 *
 * All fields are readable and writable at runtime via the IPC set_pos_config
 * command and the iPhone app.  Initialised from the _DEFAULT macros above.
 *
 * Zone thresholds (ticks): A > B > C, with D being the tightest deadband
 * (error inside zone C).  scale_* divides the raw tick error to produce a
 * lean-angle bias in degrees.  vel_scale_* divides the 100 ms tick velocity
 * for damping / back-EMF compensation.
 */
typedef struct
{
    int32_t zone_a; // Outer zone threshold (ticks)
    int32_t zone_b;
    int32_t zone_c;
    float scale_a; // Tick-error → lean-angle divisor, zone A
    float scale_b;
    float scale_c;
    float scale_d;           // Inside zone C (tightest hold)
    float vel_scale_stop;    // Velocity damp divisor when holding
    float vel_scale_move;    // Back-EMF comp divisor when driving
    float vel_scale_turning; // Reduces turning authority at speed (position-hold-style)
    int32_t stopped_vel;     // Ticks/100ms threshold for "stopped" detection
    float max_correction;    // Maximum lean-angle correction position hold may inject (deg)
    float max_angle_rate;    // Max correction change per main loop tick (deg/tick)
                             // Rate-limits position hold output to prevent slamming theta_ref.
                             // reference implementation uses 1°/loop at 500Hz ≈ 5°/loop at 100Hz.
    /* Which velocity estimate drives the damping term:
     *   0 = RoboClaw speed registers (polled every POS_VEL_PERIOD_MS and held)
     *   1 = least-squares over POS_VEL_WIN_SHORT (6 ticks, ~25 ms lag)
     *   2 = mid   (12 ticks, ~55 ms)
     *   3 = long  (20 ticks, ~95 ms)
     *
     * Runtime-selectable because it is a PHASE choice and phase is what limits
     * this loop: vel_damp supplies ~78% of the lean command, and with the
     * RoboClaw source leading position by only +77 deg (ideal +90) there is no
     * good vel_scale_stop -- 7 is underdamped, 5 is the optimum, 3.5 pins the
     * lean command at its clamp. Comparing sources needed a rebuild per trial,
     * which is why this is a knob now.
     *
     * The sources are NOT the same scale: enc_vel_mid measured 1.255x
     * vel_claw over a 95 s run (r=0.934), because the RoboClaw's own averaging
     * plus the 40 ms hold attenuates peaks. So vel_scale_stop must be scaled by
     * the same factor when changing this, or the damping GAIN changes too and
     * the experiment tells you nothing. */
    int32_t vel_src;

    /* Corner frequency (Hz) of a first-order low-pass on the VELOCITY DAMPING
     * term only. 0 disables it.
     *
     * The damping term is a derivative, and an unfiltered derivative of a
     * 1-tick-quantised encoder is a noise amplifier. Measured on this bot: the
     * position hold is stable at 0.3 Hz but runs away at ~1.6 Hz, where the
     * RoboClaw velocity estimate's ~100 ms lag has cost 58 deg of phase and the
     * "damping" reinforces instead of opposing. 1.6 Hz is also where the pitch
     * loop's own resonance sits, so it has something to feed.
     *
     * Neither gain nor a faster estimate fixes it -- both were tried:
     *   vel_scale_stop 7   (less gain)  loses the 0.3 Hz damping, wanders
     *   vel_scale_stop 3.5 (more gain)  8.7x runaway, lean pinned at the clamp
     *   vel_src 2 (55 ms lag)           unstable
     *   vel_src 1 (25 ms lag)           worse still -- shorter window, more noise
     * The one thing every result agrees on is that SMOOTHER is better, which is
     * what this is. A 1 Hz corner keeps 96% of the term's authority at 0.3 Hz
     * where it damps, and removes half of it at 1.6 Hz where it does not. */
    float vel_damp_fc;

    /* Hard cap (degrees) on how much lean the velocity damping term alone may
     * command. 0 = unlimited (the original behaviour).
     *
     * The position term is rate-limited by max_angle_rate; vel_damp is
     * subtracted AFTER that limiter and has neither a rate limit nor a cap, so
     * a velocity spike reaches theta_ref in one tick. Measured: velDamp peaks
     * at 1.44 deg in steady hold and hit 5.02 deg during a push-induced
     * runaway. A cap near 2.0 truncates the runaway and leaves normal operation
     * untouched -- which is why this is preferable to lowering the gain, which
     * degrades the hold everywhere to fix behaviour that only occurs on
     * excursions. */
    float vel_damp_max;

    /* ── Integral term on the position hold ──────────────────────────────────
     * pos_ki  : degrees of lean per (tick of error x second). 0 disables.
     * pos_i_max: hard clamp on the integral's contribution, in degrees.
     *
     * The hold is proportional-only, so the ONLY way it can command the standing
     * lean this bot needs (its CG is not over the axle) is to sit off-target:
     * lean = err / scale_d, so 0.5 deg of required lean costs 30 ticks = 1.6 in
     * of permanent offset. Measured three times in one evening at +0.25, -0.09
     * and -0.54 deg, because CG shift, battery position and IMU drift all move
     * it. Chasing it with theta_trim works once and is stale by the next run.
     *
     * The integral absorbs whatever standing lean is required and drives the
     * offset to zero on its own. It is deliberately slow -- it exists to cancel
     * a constant, not to help with disturbances, which is what the proportional
     * and damping terms are for.
     *
     * Anti-windup, all three of which matter on a robot that gets picked up:
     *   - contribution clamped to +/- pos_i_max
     *   - integration STOPS while the lean command is saturated at
     *     max_correction (integrating into a limit is how you get a lurch when
     *     the limit releases)
     *   - reset to zero on disarm, on out-of-bounds, and whenever the target is
     *     re-snapped to the current position */
    float pos_ki;
    float pos_i_max;

    int32_t pos_deadband;    // Ticks of position error inside which the hold
                             // stops PUSHING and applies damping only. Was a
                             // literal 2 in robot.c -- 2.6 mm, far tighter than
                             // anything the operator cares about, so the loop
                             // never rested and hunted around the target. Set
                             // it to roughly half the tolerance you actually
                             // want and the limit cycle loses its driver.
    int back_to_spot;        // 1 = full zone-based hold (A/B/C/D);
                             // 0 = only correct inside zone_c (loose hold, position hold mode)

    /* ── how the drive stick is interpreted ──────────────────────────────
     *
     * DRIVE_MODE_LEAN (0, the original): the stick sets theta_ref directly, so
     * it commands ACCELERATION. There is no stick position meaning "stop" --
     * centring it commands "stand upright", which does not shed the momentum
     * already there. The operator has to brake manually with reverse stick and
     * judge the reversal by eye.
     *
     * DRIVE_MODE_TARGET (1): the stick advances enc_pos_target at drive_rate,
     * so it commands VELOCITY. Position hold owns theta_ref throughout and lean
     * becomes an output. Centring the stick stops the target advancing; the bot
     * coasts past it, position error builds, and the controller produces the
     * braking lean by itself. Stopping needs no skill.
     */
    int drive_mode;          // 0 = lean (legacy), 1 = position target
    float drive_rate;        // ticks/sec the target advances at full stick
    int32_t runaway_limit;   // max |enc_pos_target - enc_pos| in ticks. Without
                             // this, a blocked or lifted wheel lets the target
                             // run away and the debt discharges violently when
                             // traction returns.
} pos_config_t;

#define DRIVE_MODE_LEAN   0
#define DRIVE_MODE_TARGET 1

// ============================================================================
// WHEEL & ENCODER PHYSICAL CONSTANTS
// RS-555 brushed DC, 5.2:1 planetary gearbox, Hall effect quadrature encoder
//   Motor:   1,150 RPM no-load @ 12V, stall torque 7.9 kg·cm @ 9.2A
//   Encoder: 28 PPR pre-gearbox → 145.1 PPR at output shaft
//   Wheel:   155mm diameter (patented)
//
// Change WHEEL_DIAMETER_MM if you swap wheels — everything else recalculates.
// ============================================================================
#define MOTOR_NO_LOAD_RPM 1150.0f                                    // RPM @ 12V no load
#define GEAR_RATIO 5.2f                                              // (1 + 46/11)
#define ENCODER_PPR_MOTOR 28.0f                                      // pulses/rev at motor shaft
#define ENCODER_TICKS_PER_REV 145.1f                                 // PPR at output shaft (GEAR_RATIO * 28)
#define WHEEL_DIAMETER_MM 155.0f                                     // ← change here if you swap wheels
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159265f)     // ~487mm
#define MM_PER_TICK (WHEEL_CIRCUMFERENCE_MM / ENCODER_TICKS_PER_REV) // ~3.36 mm

// Derived QPPS ceiling — theoretical max encoder speed at no-load full throttle
// 1150 RPM / 60 * 145.1 PPR = ~2781 QPPS.  Use ~90% for headroom.
#define QPPS_NO_LOAD ((MOTOR_NO_LOAD_RPM / 60.0f) * ENCODER_TICKS_PER_REV) // ~2781
#define QPPS_RATED 2500                                                    // Conservative working value (~90% of no-load)

// Motor HAL drive modes (mirror of motor_hal_roboclaw.c defines)
#define MOTOR_HAL_MODE_DUTY 0           // Raw PWM duty — no encoder feedback required
#define MOTOR_HAL_MODE_VELOCITY 1       // Closed-loop speed  (MIXEDSPEED,      cmd 37)
#define MOTOR_HAL_MODE_VELOCITY_ACCEL 2 // Closed-loop speed + accel ramp (cmd 40)

// motor_config defaults — override at runtime via set_motor_config IPC
#define MOTOR_HAL_MODE_DEFAULT MOTOR_HAL_MODE_DUTY
#define MOTOR_QPPS_MAX_DEFAULT QPPS_RATED         // 2500 QPPS
#define MOTOR_ACCEL_QPPS_DEFAULT (QPPS_RATED * 2) // 5000 QPPS/s ≈ 0.5 s ramp
#define MOTOR_BAUD_DEFAULT 460800

// RoboClaw internal velocity-PID defaults (fixed-point × 65536 on the wire)
// Factory defaults per BasicMicro: Kp=1.0  Ki=0.5  Kd=0.25
// Only active in modes 1 (velocity) and 2 (velocity+accel).
#define MOTOR_CLAW_KP_DEFAULT 1.0f
#define MOTOR_CLAW_KI_DEFAULT 0.5f
#define MOTOR_CLAW_KD_DEFAULT 0.25f

// TODO: these shouldn't be floats
#define MOTOR_ENC_POL_L -1.0f
#define MOTOR_ENC_POL_R -1.0f
#define MOTOR_MOT_POL_L 1.0f
#define MOTOR_MOT_POL_R 1.0f

/**
 * @brief Runtime-tunable RoboClaw drive mode and velocity parameters.
 *
 * mode       — selects the RoboClaw command used by motor_hal_set_both():
 *              0 = raw PWM duty (MIXEDDUTY, cmd 34)
 *              1 = closed-loop velocity (MIXEDSPEED, cmd 37)
 *              2 = closed-loop velocity + acceleration ramp (MIXEDSPEEDACCEL, cmd 40)
 *
 * qpps_max   — top motor speed in encoder pulses/second at ±1.0 normalised output.
 *              Measure with Basic Micro Motion Studio while commanding 100% duty.
 *
 * accel_qpps — acceleration ramp rate (pulses/s²), mode 2 only.
 *              ~2×qpps_max gives a ≈0.5 s ramp; increase for snappier response.
 *
 * pol_l/pol_r — polarity flip per motor (+1.0 or -1.0). Flip if a motor runs
 *              backwards relative to the robot's forward direction.
 *
 * All fields are readable/writable at runtime via the IPC set_motor_config
 * command and persisted by save_pid.
 */
typedef struct
{
    int mode;        // MOTOR_HAL_MODE_DUTY / _VELOCITY / _VELOCITY_ACCEL
    int qpps_max;    // Max encoder speed (pulses/s) at full throttle
    int accel_qpps;  // Acceleration ramp rate (pulses/s²), mode 2 only
    float pol_l;     // Left  motor polarity: +1.0 or -1.0
    float pol_r;     // Right motor polarity: +1.0 or -1.0
    float enc_pol_l; // Left  encoder polarity: +1.0 or -1.0
    float enc_pol_r; // Right encoder polarity: +1.0 or -1.0

    // RoboClaw internal velocity PID — only active in modes 1 and 2.
    float claw_kp;
    float claw_ki;
    float claw_kd;

    // Serial baud rate for the RoboClaw connection on the BBB.
    // Changing this via set_motor_config causes live reconnection.
    int baud;
} motor_config_t;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Generic input packet from an external UART source.
 *
 * Wire format (ASCII, newline-terminated):
 *   PKT,<x>,<y>,<conf>\n
 * where x/y are normalised floats (-1..+1) and conf is 0..1.
 *
 * uart_input_get() fills this and returns 1 when fresh data is available.
 */
typedef struct
{
    float x;               // Lateral / steering command  (-1 to +1)
    float y;               // Forward / drive command     (-1 to +1)
    float confidence;      // Source confidence           ( 0 to  1)
    uint64_t timestamp_ns; // rc_nanos_since_boot() at receipt
    int valid;             // 1 = fresh, 0 = stale / no signal
} input_packet_t;

/**
 * @brief PID controller state
 */
typedef struct
{
    float kp, ki, kd;
    float integrator;
    float prev_error;
    float dt;
    float integrator_max;

    // Last computed terms, recorded by pid_update() for telemetry.
    // Telemetry must read these rather than recomputing from kp/ki/integrator:
    // the derivative depends on the previous error, which is overwritten each
    // tick, so it cannot be reconstructed after the fact.
    float last_p_term;
    float last_i_term;
    float last_d_term;
    float last_output;
} pid_controller_t;

/**
 * @brief Robot operating mode
 */
typedef enum
{
    MODE_IDLE,      // Motors off
    MODE_BALANCE,   // Balance only, no movement
    MODE_EXT_INPUT, // External UART packet drives the bot
    MODE_MANUAL     // Xbox / SBUS controller
} robot_mode_t;

/**
 * @brief Complete robot state
 */
typedef struct
{
    // IMU
    float theta;     // Body pitch angle  (deg)
    float theta_dot; // Body pitch rate   (deg/s)
    float phi;       // Body roll angle   (deg)
    float psi;       // Body yaw angle    (deg)

    // Encoders
    int32_t enc_left; // Raw encoder ticks (always updated)
    int32_t enc_right;
    float phi_left;  // Left wheel angle  (deg)
    float phi_right; // Right wheel angle (deg)

    // position hold controller (encoder-tick based)
    int32_t enc_pos;        // Sum of left+right encoder ticks (position)
    int32_t enc_pos_target; // Target tick position (held when stick is centered)
    float enc_velocity;     // Tick velocity (ticks per 100 ms window)
    float enc_velocity_raw; // The candidate NOT in control (see robot.c)
    float enc_vel_lsq_mid;  // candidate: mid window,  logged only
    float enc_vel_lsq_long; // candidate: long window, logged only
    int enc_vel_reset;      // Set to 1 by ipc_server after zero_encoders; cleared by robot.c

    // Legacy degree-based position (kept for telemetry)
    float pos;          // avg wheel angle (deg)
    float pos_setpoint; // position setpoint (deg) — unused when position enabled

    // Control references
    float theta_ref;    // Desired body angle  (deg)
    float pose_lean;    // Commanded lean for observation (deg). DISTINCT from
                        // theta_offset: theta_offset defines where upright IS,
                        // pose_lean deliberately leans AWAY from upright so you
                        // can watch the bot creep at a known angle. Ramped in and
                        // out at max_angle_rate; cleared on disarm.
    float theta_offset; // Balance point trim  (deg) — tunable from iPhone
    float yaw;          // Heading target, degrees of phi_diff (was 'steering')

    // steering latch — when the drive stick returns to centre, steering holds
    // the phi_diff at that moment rather than fighting back to zero.
    float steering_latch; // phi_diff value latched at stick-centre transition
    int yaw_latched; // 1 = latch is active (stick centred), 0 = driving

    // External UART input (used only in MODE_EXT_INPUT)
    input_packet_t ext_input;

    // position hold controller internal signals (for telemetry)
    float pos_correction; // lean angle from position error (deg)
    float pos_vel_damp;       // lean angle from velocity damping (deg)
    float pos_i_term;         // lean angle from the integral (deg)
    float pos_output; // final rate-limited, clamped correction injected (deg)
    float pos_scale;   // zone divisor used this tick (0 = deadband, no correction).
                             // position hold is gain-scheduled, so "which zone am I in" is the
                             // closest thing it has to a gain — log it explicitly.

    robot_mode_t mode;
    int trying;
    int armed;         // 0 = disarmed, 1 = armed
    int estop_latched; // 1 = RoboClaw estop latched, needs WriteNVM reset
} robot_state_t;

// ============================================================================
// GLOBAL STATE (defined in robot.c)
// ============================================================================

extern robot_state_t state;
extern rc_mpu_data_t mpu_data;
extern pid_controller_t pitch_pid;
extern pid_controller_t yaw_pid;
extern debug_config_t g_debug_config;
extern telemetry_data_t g_telemetry_data;
extern pos_config_t g_pos_config;
extern motor_config_t g_motor_config;
// Mirrors robot_config_t.arm_at_boot -- robot_config_apply() copies it in,
// robot_run()'s main loop consumes it once on its first iteration (after
// the DMP thread has real theta data, not before), then it's inert until
// the next restart. set_arm_at_boot persists the preference immediately.
extern int g_arm_at_boot;

// The one out-of-bounds angle (degrees) that both imu_interrupt()'s actuation
// gate and robot_run()'s hard motor-cutoff read -- see robot.c. Mirrors
// robot_config_t.oob_angle_deg; set_oob_angle applies and persists it
// immediately, unlike arm_at_boot which only takes effect on next boot.
extern float g_oob_angle_deg;

// ============================================================================
// PID (pid.c)
// ============================================================================

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt);
float pid_update(pid_controller_t *pid, float setpoint, float measurement);
void pid_reset(pid_controller_t *pid);
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

// ============================================================================
// GENERIC UART INPUT (uart_input.c)
// ============================================================================

/**
 * @brief Open a UART and start the background reader thread.
 *
 * @param device      e.g. "/dev/ttyO1"
 * @param baud        e.g. 115200
 * @param timeout_ms  Age after which a packet is marked stale
 * @return 0 on success, -1 on error
 */
int uart_input_init(const char *device, int baud, int timeout_ms);
int uart_input_get(input_packet_t *pkt); // 1=valid, 0=stale
void uart_input_cleanup(void);

// ============================================================================
// ROBOT CONTROL (robot.c)
// ============================================================================

int robot_init(void);
void robot_run(void);
void robot_cleanup(void);

// ============================================================================
// IPC SERVER (ipc_server.c)
// ============================================================================

int ipc_server_init(void);
void ipc_server_cleanup(void);

/* The stream is split into three messages rather than one combined packet:
 *
 *   telemetry  dynamic state (imu, encoders, controllers, motors). Sent at
 *              rates.pid_states.
 *   rc         raw + decoded SBUS. Sent at rates.rc, which is much higher --
 *              the receiver produces a frame every ~7 ms, so bundling this
 *              into the 10 Hz telemetry packet threw away 13 of every 14
 *              samples and no amount of client-side work could recover them.
 *   config     motor_config / pos_config / sbus_config. These only change on
 *              command, so they are sent on connect and on change instead of
 *              being retransmitted in every packet.
 *
 * All three are newline-terminated JSON objects carrying a "type" field, on the
 * same socket. A reader must dispatch on "type" and ignore types it does not
 * know, so this can be extended without breaking existing clients. */
void ipc_broadcast_telemetry(void);
void ipc_broadcast_rc(void);
void ipc_broadcast_config(void);

/* Mark the config packet as needing a resend. Call from anywhere that changes
 * motor_config, pos_config or sbus_config. Cheap -- sets a flag; the control
 * loop does the send via ipc_broadcast_config_if_dirty(). */
void ipc_config_touch(void);

/* Loop-rate capture to /tmp/bbot.csv. The websocket telemetry is 20 Hz against
 * a 100 Hz loop, so anything above 10 Hz aliases; this is the only honest view
 * of what the derivative term is actually doing. RAM buffer during the run,
 * file written when it fills. Fixed path, overwritten each time. */
int looplog_start(int seconds);
int looplog_active(void);
void ipc_broadcast_config_if_dirty(void);

/* Worst telemetry-drop count across connected clients. Reported in the
 * telemetry packet so a stalled bridge is visible as a number rather than
 * inferred from gaps in a graph. */
unsigned long ipc_get_tx_drops(void);

// ============================================================================
// TELEMETRY (telemetry.c)
// ============================================================================

int telemetry_init(void);
void telemetry_update(void);
void telemetry_get_config_description(char *buf, size_t size);
void telemetry_print_summary(void);

// ============================================================================
// PID CONFIG FILE (pid_config.c)
// ============================================================================

/* robot_config_t is defined below, after imu_offsets_t. */

// ============================================================================
// POSITION CONTROLLER CONFIG (pid_config.c)
// ============================================================================

/**
 * @brief Apply a pos_config_t to the global g_pos_config.
 * Called from IPC set_pos_config handler and on startup.
 */
void pos_config_apply(const pos_config_t *cfg);

/**
 * @brief Populate *cfg from the current g_pos_config values.
 * Used by save_pid to persist position params alongside PID gains.
 */

/**
 * @brief Save pos_config to file (appended section in pidconfig.txt).
 */

/**
 * @brief Load pos_config from file, or fill defaults if section absent.
 */

// ============================================================================
// MOTOR CONFIG (pid_config.c)
// ============================================================================

/** @brief Apply a motor_config_t to the global g_motor_config. */
void motor_config_apply(const motor_config_t *cfg);

/** @brief Populate *cfg from the current g_motor_config values. */

/** @brief Save motor_config to file (appended section in pidconfig.txt). */

/** @brief Load motor_config from file, or fill defaults if section absent. */

// ============================================================================
// XBOX CONTROLLER (input_xbox.c)
// ============================================================================

int xbox_init(const char *device);
int xbox_update(void);
float xbox_get_drive(void);
float xbox_get_turn(void);
int xbox_get_arm_button(void);
void xbox_cleanup(void);

// ============================================================================
// SBUS INPUT (input_sbus.c)
// ============================================================================

int sbus_init(const char *device);
int sbus_update(void);
void sbus_cleanup(void);
float sbus_get_drive(void);
float sbus_get_turn(void);
int sbus_get_arm(void);
int sbus_get_kill(void);
int sbus_get_speed_mode(void);
bool sbus_get_failsafe(void);
bool sbus_is_connected(void);
bool sbus_drive_armed(void);
float sbus_get_aux1(void);
float sbus_get_aux2(void);
int sbus_get_sw_c(void);
int sbus_get_sw_e(void);
bool sbus_get_sw_f(void);
uint16_t sbus_get_channel_raw(int ch);
float sbus_get_channel_float(int ch);

// ============================================================================
// IMU ORIENTATION (fixed for BeagleBone Blue mounted vertically)
//
// BBB axes when bot stands upright:
//   +X = down, +Z = forward, +Y = left
//
// Balance axis: forward/back lean = rotation around Y = TB_ROLL_Y
// Upright resting angle stored as pitch_offset (radians, set via zero_imu).
//
// theta (deg) = (TB_ROLL_Y - pitch_offset) * RAD_TO_DEG
// ============================================================================

typedef struct
{
    float pitch_offset;     // angle when upright (degrees)
    float yaw_offset;       // yaw at heading zero (degrees)
    float pitch_dot_offset; // gyro Y bias (deg/s) — corrects constant drift
    int pitch_axis;         // gravity axis for pitch: 0=X  1=Y(default)  2=Z
} imu_offsets_t;

/**
 * @brief Which SBUS channels drive the robot, and how hard.
 *
 * Channel numbers are 1-based, matching what the transmitter displays.
 *
 * drive_scale exists because MAX_THETA_REF is 17 degrees: at full stick the
 * bot commands a 17-degree lean, which on this chassis is enough to throw
 * itself over before the balance loop can catch it. Scale it right down while
 * learning the machine.
 *
 * require_center is the important one. A ratcheted throttle stick (CH3) rests
 * at the BOTTOM of its travel, not the middle — read as a bipolar command that
 * is full reverse. With this set, drive stays at zero until the channel has
 * been seen near centre at least once since the link came up, so the bot
 * cannot lurch the instant the receiver binds.
 */
typedef struct
{
    int drive_channel;   /* 1-based. CH2 (Ele) springs back; CH3 (Thr) stays put */
    int turn_channel;    /* 1-based. Normally CH1 (Ail)                          */
    float drive_scale;   /* multiplies the normalised stick, 0..1                */
    float turn_scale;
    int drive_invert;    /* 1 = stick forward gives positive drive               */
    int turn_invert;
    float turn_rate;     /* deg of wheel-differential per second at full stick.
                          * The steering loop tracks phi_diff = (phi_R - phi_L)/2
                          * in degrees, so this is a rate in those units, NOT a
                          * chassis yaw rate. A full chassis spin is roughly
                          * (wheelbase/wheel_circumference) * 360 of phi_diff. */
    float deadband;      /* fraction of half-travel treated as centre            */
    int require_center;  /* 1 = refuse drive until the stick has been centred    */
} sbus_config_t;

extern sbus_config_t g_sbus_config;

typedef struct
{
    float kp, ki, kd;
} pid_gains_t;

/**
 * @brief Everything tunable on the robot, in one struct backed by one file.
 *
 * Supersedes pid_config_file_t + pos_config_t + motor_config_t + imu_offsets_t
 * being loaded and saved independently from two files in three formats. See
 * robot_config.c for why that arrangement could not be made safe.
 */
typedef struct
{
    pid_gains_t pitch;    /* was balance — pitch angle -> duty, real PID   */
    pid_gains_t yaw;        /* was steering — wheel diff -> duty, real PID   */
    pos_config_t position;  /* was position — zone-scheduled hold, NOT a PID    */
    motor_config_t motor;
    imu_offsets_t imu;
    sbus_config_t sbus;
    float theta_trim;       /* was balance_angle; applied as state.theta_offset */
    int arm_at_boot;        /* 1 = auto-arm on startup instead of waiting for ARM */
    float oob_angle_deg;    /* hard safety cutoff -- see g_oob_angle_deg */
} robot_config_t;

void robot_config_defaults(robot_config_t *c);
int robot_config_load(const char *path, robot_config_t *c);
int robot_config_save(const char *path, const robot_config_t *c);
void robot_config_get_current(robot_config_t *c);
void robot_config_apply(const robot_config_t *c);

/** Snapshot every live global and rewrite the whole file. The only way to save. */
int robot_config_save_current(const char *path);

/** Load robot.conf, or build it from the legacy pidconfig.txt + IMU file once. */
int robot_config_load_or_migrate(const char *path, robot_config_t *c);


typedef struct
{
    float pitch, yaw, roll;
    float pitch_dot, yaw_dot, roll_dot;
    float accel_x, accel_y, accel_z;
} imu_transform_t;

extern imu_offsets_t g_imu_offsets;

typedef struct
{
    bool pitch;
    bool position;
    bool yaw;
} controller_enables_t;
extern controller_enables_t g_controllers;

void imu_offsets_calibrate(const rc_mpu_data_t *raw, imu_offsets_t *offsets);
void imu_apply_transform(const rc_mpu_data_t *raw, imu_transform_t *out,
                         const imu_offsets_t *offsets);

// ============================================================================
// UTILITY MACROS
// ============================================================================

#ifndef rc_saturate_float
#define rc_saturate_float(val, mn, mx) \
    do                                 \
    {                                  \
        if (*(val) < (mn))             \
            *(val) = (mn);             \
        else if (*(val) > (mx))        \
            *(val) = (mx);             \
    } while (0)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif

#endif /* BALANCE_BOT_H */