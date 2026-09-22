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
 * @file robot.c
 * @brief Main robot control loop with telemetry and IPC integration
 */

#include "balance_bot.h"
#include "motor_hal.h"
#include "display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <pthread.h>
#include <roboclaw_estop.h>

/* Process state — used by rc_get_state()/rc_set_state() in rc_compat.h */
volatile rc_state_t g_rc_state = UNINITIALIZED;

// Global state
robot_state_t state = {0};
rc_mpu_data_t mpu_data;
pid_controller_t pitch_pid, yaw_pid;

/* Heading target, in the same units as phi_diff: degrees of differential wheel
 * rotation. File scope rather than a function-local static ONLY so that
 * robot_reset_heading() below can clear it. Nothing else may touch it.
 *
 * Why that matters: zero_encoders clears phi_left/phi_right, which is the
 * steering loop's MEASUREMENT. While this kept its old value the setpoint
 * survived the reset, so the error stepped from ~0 to the whole accumulated
 * heading in one tick and the bot spun hard to pay off a debt that no longer
 * existed. Clearing state.yaw alone was not enough -- the armed + SBUS branch
 * reassigns state.yaw = yaw_target every tick, so it came straight back. */
static float yaw_target = 0.0f;

/* Clear the steering loop's heading target and latch. Call whenever the wheel
 * positions that the measurement is derived from are reset out from under it,
 * so setpoint and measurement are zeroed together and no step appears. */
/* Last tick's yaw rates, both in phi_diff deg/s, for the log. enc is what the
 * loop differentiates today; gyro is what it can use instead. Logging both is
 * how yaw_gyro_scale gets calibrated without any guessing. */
static float yaw_rate_gyro = 0.0f;
static float yaw_rate_enc = 0.0f;
static float yaw_phi_prev = 0.0f;
static int yaw_phi_primed = 0;

void robot_reset_heading(void)
{
    yaw_target = 0.0f;
    state.yaw = 0.0f;
    state.steering_latch = 0.0f;
    state.yaw_latched = 0;
    /* The integral has been winding against the old heading; carrying it over
     * would reintroduce a fraction of the same kick. */
    pid_reset(&yaw_pid);
    /* phi_left/right are about to be (or have just been) zeroed, so the stored
     * previous phi_diff is meaningless -- differencing against it would put one
     * enormous rate spike through the D term on the next tick. */
    yaw_phi_primed = 0;
    yaw_rate_enc = 0.0f;
    yaw_rate_gyro = 0.0f;
}

/* Low-pass state for the position hold's velocity damping term. File scope so
 * the disarm path can clear it. */
static float vel_damp_filt = 0.0f;

/* Integral accumulator for the position hold, in DEGREES of lean (not raw
 * tick-seconds) so pos_i_max is a limit you can reason about directly. */
static float pos_integ = 0.0f;

/* Approach profile ("the carrot"). Position hold chases this, not
 * enc_pos_target -- the target itself.
 * It walks toward home at return_rate and is never allowed more than lead_max
 * ticks from the wheels. So a push is ARRESTED first (carrot held near the
 * wheels: small spring, the damping does the stopping) and then the bot is
 * WALKED home at a steady pace, with the error never above lead_max.
 * lead_max = 0 disables it: the controller chases home directly, as before. */
static float carrot = 0.0f;
static float carrot_v = 0.0f;   /* carrot's PLANNED speed, enc_pos ticks/s (signed) */
static int carrot_valid = 0;
/* The carrot has reached the target and parks there until a new one is set.
 * Without this the lead clamp kept shoving it back out to (wheels +/- lead)
 * after an overshoot, the trapezoid re-planned a full-speed run from wherever
 * it landed, and the bot got a fresh acceleration command every time it blew
 * through the target -- a growing oscillation. Log 2026-09-20
 * bbot_ring_1789928811195: overshoot 176 -> 211 -> 440 ticks over three
 * half-cycles at ~0.22 Hz, with pos_carrotVel visibly flipping +89.7 -> -11.3
 * -> -30.2 mid-move as the runaway rewrote the plan. */
static int carrot_arrived = 0;
/* The target the running plan was built for. A move command just assigns
 * state.enc_pos_target (ipc_server.c "nudge fwd", among ~10 other sites) and
 * cannot be expected to reach in here, so the plan notices the change itself.
 * Without this carrot_arrived latched on the first tick -- standing still IS
 * "arrived" -- and never cleared, so every later move ran with carrot_v = 0:
 * no plan, no feed-forward, and the damper fighting the whole speed. The bot
 * crept at the ~33 ticks/s where a 1 deg spring balances vel_scale_stop, which
 * looks exactly like not moving at all. */
static int32_t carrot_home_prev = 0;

static float pos_last_correction = 0.0f;

controller_enables_t g_controllers = {
    .pitch = true,
    .position = true,
    .yaw = true,
};

// Runtime-tunable position controller parameters (initialised in robot_init)
pos_config_t g_pos_config;

// Transmitter mapping and gains (initialised from robot.conf via
// robot_config_apply). Defined here rather than in input_sbus.c because
// robot_config.c persists it and must link against it. Read by input_sbus.c for
// channel mapping/scaling and by the steering integrator below for turn_rate.
sbus_config_t g_sbus_config;

// Runtime-tunable motor/RoboClaw drive parameters (initialised in robot_init)
motor_config_t g_motor_config;
int g_arm_at_boot = 0;

// Count of DMP samples delivered since start. Written only by imu_interrupt,
// read by the main loop; volatile because those are different contexts and the
// compiler must not cache it across the wait below.
//
// This exists because "the IMU is initialised" and "the IMU has produced a
// reading" are different facts, and arm_at_boot was built on the first one
// while needing the second. robot_init() sets the DMP going and returns; the
// first interrupt lands ~300ms later. In that window state.theta is still its
// initialised 0.0, which reads as perfectly upright and passes every bounds
// check there is.
volatile uint32_t g_imu_samples = 0;

// Debug configuration (defined in debug_config.h, initialized in main.c)
debug_config_t g_debug_config;

// Telemetry counters
static uint64_t telemetry_counter = 0;
/* Absolute due-times (microseconds since boot) for the two outbound streams.
 * Advanced by exactly one period per send so the average rate is correct even
 * when the period is not a whole number of loop ticks -- see the control loop. */
static uint64_t next_telemetry_us = 0;
static uint64_t next_rc_us = 0;

// Motor duty tracking (for telemetry)
static float last_left_duty = 0.0f;
static float last_right_duty = 0.0f;

// Motor output handoff: ISR writes, main loop applies to hardware.
// volatile prevents compiler optimizing away cross-context reads/writes.
static volatile float pending_left_duty = 0.0f;
static volatile float pending_right_duty = 0.0f;
static volatile int motor_output_ready = 0;

// The one out-of-bounds angle both the ISR's actuation gate and robot_run()'s
// hard cutoff must agree on. Used to live as two separate "15.0f" literals --
// the ISR's gate never actually had one, which is how arming at 18deg briefly
// drove the wheels every ISR tick until the main loop's cutoff caught up.
// Runtime-tunable now (robot.conf [system] oob_angle_deg) rather than a
// compile-time constant -- 15.0f here is only the default before a config
// loads or if the key is absent.
float g_oob_angle_deg = 15.0f;

/* 0 = steering derivative from the encoder staircase (original behaviour).
 * Nonzero = from gyro Z, multiplied by this to reach phi_diff units. The sign
 * is part of it. See YAW_GYRO_SCALE_DEFAULT in balance_bot.h. */
float g_yaw_gyro_scale = YAW_GYRO_SCALE_DEFAULT;

/* Stand-up kick. See KICK_ASSIST_DEFAULT in balance_bot.h for the fencing. */
int g_kick_assist = KICK_ASSIST_DEFAULT;
float g_kick_max_deg = KICK_MAX_DEG_DEFAULT;
float g_kick_duty = KICK_DUTY_DEFAULT;
int g_kick_timeout_ms = KICK_TIMEOUT_MS_DEFAULT;
volatile int g_kick_request = 0;   /* dashboard hold-to-kick: -1, 0, +1 */
volatile uint64_t g_kick_request_us = 0;

/* Returns the kick direction (-1, 0, +1) this tick, or 0 for "do not kick".
 * Holds the centre-seen latch and the timeout, so it must be called exactly
 * once per main-loop tick while out of bounds. */
static int kick_direction(float eff_angle)
{
    static int centred_since_last = 1;  /* must see centre before each kick */
    static uint64_t kick_started_us = 0;

    /* A request from either source. The dashboard button sets g_kick_request
     * directly; the transmitter uses the drive stick past a deadband. */
    float stick = 0.0f;
    if (sbus_is_connected() && !sbus_get_failsafe())
        stick = sbus_get_drive();
    int want = 0;
    const uint64_t now_us = rc_nanos_since_boot() / 1000;
    /* The dashboard request expires unless it keeps being refreshed, so a
     * dropped websocket cannot leave the wheels driving. */
    const int dash = (g_kick_request != 0 &&
                      (now_us - g_kick_request_us) < KICK_REQUEST_STALE_US)
                         ? g_kick_request
                         : 0;
    if (dash != 0)
        want = (dash > 0) ? 1 : -1;
    else if (fabsf(stick) > KICK_STICK_DEADBAND)
        want = (stick > 0.0f) ? 1 : -1;

    if (want == 0)
    {
        /* Released: re-arm the latch and forget the timer. */
        centred_since_last = 1;
        kick_started_us = 0;
        return 0;
    }

    if (!g_kick_assist || !state.armed || g_imu_stale)
        return 0;
    /* Inside oob the balance loop owns the wheels; above kick_max_deg it is
     * lying down and a kick only throws it. */
    if (eff_angle <= g_oob_angle_deg || eff_angle > g_kick_max_deg)
        return 0;
    if (!centred_since_last)
        return 0;   /* held over from a previous kick -- recentre first */

    if (kick_started_us == 0)
    {
        kick_started_us = now_us;
        LOG_WARN("kick: standing up, dir=%+d eff=%.1f deg", want, eff_angle);
    }
    else if ((now_us - kick_started_us) > (uint64_t)g_kick_timeout_ms * 1000ULL)
    {
        /* Long enough. Either it is stuck or the stick is stuck; either way
         * stop, and refuse to start again until it has been released. */
        centred_since_last = 0;
        kick_started_us = 0;
        LOG_WARN("kick: timed out after %d ms, release and retry", g_kick_timeout_ms);
        return 0;
    }
    return want;
}


/* Sensor watchdog: while armed, theta must move by at least this much within
 * this long, or the reading is stale and the motors are cut. The threshold is
 * well below real IMU noise on purpose -- this is meant to catch a frozen
 * feed, not a quiet one. */
#define IMU_STALE_EPS_DEG 0.001f
#define IMU_STALE_US 300000ULL
volatile int g_imu_stale = 0;

/**
 * @brief IMU interrupt callback — runs at SAMPLE_RATE_HZ.
 *
 * Kept minimal: read angles from the already-populated DMP struct,
 * run PID, hand off outputs to main loop via pending_* variables.
 * No I/O, no UART, no blocking calls.
 */
static void imu_interrupt(void)
{
    // mpu_data is populated by the DMP library before this callback fires.
    // Apply mounting transform to get angles in robot frame (degrees).
    imu_transform_t t;
    imu_apply_transform(&mpu_data, &t, &g_imu_offsets);

    state.theta = t.pitch;         // deg
    state.theta_dot = t.pitch_dot; // deg/s
    state.psi = t.yaw;             // deg
    state.psi_dot = t.yaw_dot;     // deg/s, gyro Z -- the steering loop's D source
    g_imu_samples++;               // proof that theta is a measurement

    // Ungated: PID runs every tick regardless of armed/trying, so telemetry
    // (this writes pitch_pid.last_p_term/i_term/d_term/last_output, which
    // telemetry.c reads directly) and integrator history stay live instead of
    // freezing mid-fall. Actuation is still gated below -- this only decides
    // whether the math runs, not whether the wheels move. The gate below now
    // checks eff_angle itself (not just armed/trying), so this ISR never
    // publishes real duty while out of bounds. robot_run()'s own
    // eff_angle>g_oob_angle_deg cutoff still forces motor_hal_set_both(0,0) +
    // standby(1) unconditionally on top of this, every main-loop tick, as a
    // second, independent stop for anything that reaches the hardware.
    float balance_output = 0.0f;
    float yaw_output = 0.0f;

    if (g_controllers.pitch)
    {
        balance_output = pid_update(&pitch_pid,
                                    state.theta_ref + state.theta_offset,
                                    state.theta);
    }

    if (g_controllers.yaw)
    {
        float phi_diff = (state.phi_right - state.phi_left) / 2.0f;

        /* Anti-windup: the loop may be asked to correct at most MAX_YAW_LEAD
         * degrees of heading at once. Applied to the MEASUREMENT the PID sees
         * rather than by moving state.yaw, so the real target survives a twist
         * bigger than the clamp and the robot keeps working its way back. */
        {
            const float lead = state.yaw - phi_diff;
            if (lead > MAX_YAW_LEAD)
                phi_diff = state.yaw - MAX_YAW_LEAD;
            else if (lead < -MAX_YAW_LEAD)
                phi_diff = state.yaw + MAX_YAW_LEAD;
        }

        /* Both rates, every tick, whichever one the loop is using. */
        if (!yaw_phi_primed)
        {
            yaw_phi_prev = phi_diff;
            yaw_phi_primed = 1;
        }
        yaw_rate_enc = (phi_diff - yaw_phi_prev) / yaw_pid.dt;
        yaw_phi_prev = phi_diff;
        yaw_rate_gyro = state.psi_dot * g_yaw_gyro_scale;

        /* phi_diff moves in whole encoder counts (1.2405 deg each), so
         * differencing it makes a one-tick impulse per count that swamps kp --
         * the idle yaw dither. Gyro Z measures the same rotation continuously.
         * Off by default: the scale carries the mounting sign. */
        if (g_yaw_gyro_scale != 0.0f)
            yaw_output = pid_update_rate(&yaw_pid, state.yaw, phi_diff,
                                         yaw_rate_gyro);
        else
            yaw_output = pid_update(&yaw_pid, state.yaw, phi_diff);
    }
    else
    {
        /* Was: yaw_output = state.yaw.  state.yaw is a HEADING TARGET in
         * degrees of phi_diff (see its declaration in balance_bot.h) -- a
         * setpoint, not a duty.  Feeding it straight into the differential
         * below mixes units: phi_now grows without bound as the wheels turn,
         * so within a metre of travel state.yaw is large enough to saturate
         * left/right to ±1 and the bot spins at full authority.  That is why
         * turning the yaw controller OFF made the spin worse rather than
         * removing it.  Disabled means no differential. */
        yaw_output = 0.0f;
    }

    // Motors only ACTUATE when armed, trying, AND actually in safe angle
    // range -- checked here, in the ISR, not just in robot_run()'s separate
    // cutoff. trying no longer implies "in range" (it is persistent now), so
    // this angle check is the only thing standing between an out-of-bounds
    // arm and the wheels. pid_update() above already ran either way.
    /* Sensor watchdog. A balancing robot's angle is never perfectly constant,
     * so a theta that has not moved is a dead sensor, not a steady robot. The
     * OOB check below cannot catch this -- it is watching the frozen number. */
    {
        static float last_theta = 0.0f;
        static uint64_t last_move_us = 0;
        uint64_t now = rc_nanos_since_boot() / 1000;

        /* Armed only. Parked on the kickstand theta is legitimately almost
         * constant, and checking there would trip on every boot. Armed, the
         * body is always moving -- that is what balancing is. */
        if (!state.armed)
        {
            last_move_us = 0;
            g_imu_stale = 0;
        }
        else if (last_move_us == 0 || fabsf(state.theta - last_theta) > IMU_STALE_EPS_DEG)
        {
            last_theta = state.theta;
            last_move_us = now;
            g_imu_stale = 0;
        }
        else if (now - last_move_us > IMU_STALE_US)
        {
            g_imu_stale = 1; /* main loop logs it; no I/O in here */
        }
    }

    float eff_angle = fabsf(state.theta - state.theta_offset);
    if (!state.armed || !state.trying || g_imu_stale || eff_angle > g_oob_angle_deg)
    {
        pending_left_duty = 0.0f;
        pending_right_duty = 0.0f;
        motor_output_ready = 0;
        return;
    }

    float left_duty = balance_output - yaw_output;
    float right_duty = balance_output + yaw_output;
    rc_saturate_float(&left_duty, -1.0f, 1.0f);
    rc_saturate_float(&right_duty, -1.0f, 1.0f);

    // Hand off to main loop — no motor I/O in the ISR.
    pending_left_duty = left_duty;
    pending_right_duty = right_duty;
    motor_output_ready = 1;
}

/* ── Loop-rate logging ──────────────────────────────────────────────────────
 * The websocket telemetry runs at 20 Hz while this loop runs at 100 Hz, which
 * puts Nyquist at 10 Hz. Anything the derivative term does above that -- which
 * on a noisy IMU is most of what it does -- folds back and shows up in a
 * spectrum as low-frequency content that was never there. Tuning decisions made
 * from a 20 Hz log of a 100 Hz loop are decisions made from aliases.
 *
 * So: capture into RAM at full loop rate, write the file afterwards. No file
 * I/O inside the loop at all -- one struct copy per tick, ~120 bytes memcpy.
 * Columns are byte-identical to the dashboard's CSV export so tools/analyze_tune.py
 * and every existing plot work on it unchanged. */


typedef struct
{
    uint64_t t_us; /* absolute, us since boot; re-based on write */
    float pit_sp, pit_meas, pit_err, pit_p, pit_i, pit_d, pit_out;
    int32_t enc_target, enc_pos, enc_err;
    float enc_vel, enc_vel_raw, enc_vel_mid, enc_vel_long;
    float pos_corr, vel_damp, pos_iterm, theta_adj, active_scale;
    float yaw_sp, yaw_meas, yaw_err, yaw_p, yaw_i, yaw_d, yaw_out;
    float left_duty, right_duty;
    float loop_hz;
    int32_t link_down;
    float carrot_lead;   /* carrot - enc_pos, ticks (0 when the carrot is off) */
    float carrot_vel;    /* carrot's planned speed, enc_pos ticks/s */
    float yaw_gyro_rate; /* gyro Z * yaw_gyro_scale, phi_diff deg/s */
    float yaw_enc_rate;  /* d(phi_diff)/dt differenced, phi_diff deg/s */
    float yaw_psi_dot;   /* RAW chassis yaw rate from gyro Z, deg/s */
} looplog_row_t;

/* One row of the 100 Hz log, from this tick's telemetry. Shared by the
 * one-shot/free capture and the always-on ring. */
static void looplog_fill_row(looplog_row_t *r, uint64_t now)
{
    r->t_us = now;
    r->pit_sp = g_telemetry_data.pitch.setpoint;
    r->pit_meas = g_telemetry_data.pitch.measurement;
    r->pit_err = g_telemetry_data.pitch.error;
    r->pit_p = g_telemetry_data.pitch.p_term;
    r->pit_i = g_telemetry_data.pitch.i_term;
    r->pit_d = g_telemetry_data.pitch.d_term;
    r->pit_out = g_telemetry_data.pitch.output;
    r->enc_target = g_telemetry_data.position.enc_pos_target;
    r->enc_pos = g_telemetry_data.position.enc_pos;
    r->enc_err = g_telemetry_data.position.enc_error;
    r->enc_vel = g_telemetry_data.position.enc_velocity;
    r->enc_vel_raw = g_telemetry_data.position.enc_velocity_raw;
    r->enc_vel_mid = g_telemetry_data.position.enc_vel_lsq_mid;
    r->enc_vel_long = g_telemetry_data.position.enc_vel_lsq_long;
    r->pos_corr = g_telemetry_data.position.pos_correction;
    r->vel_damp = g_telemetry_data.position.vel_damp;
    r->pos_iterm = state.pos_i_term;
    r->theta_adj = g_telemetry_data.position.theta_ref_adj;
    r->active_scale = g_telemetry_data.position.active_scale;
    r->yaw_sp = g_telemetry_data.yaw.setpoint;
    r->yaw_meas = g_telemetry_data.yaw.measurement;
    r->yaw_err = g_telemetry_data.yaw.error;
    r->yaw_p = g_telemetry_data.yaw.p_term;
    r->yaw_i = g_telemetry_data.yaw.i_term;
    r->yaw_d = g_telemetry_data.yaw.d_term;
    r->yaw_out = g_telemetry_data.yaw.output;
    r->left_duty = g_telemetry_data.motors.left_duty;
    r->right_duty = g_telemetry_data.motors.right_duty;
    /* The two columns that say whether the rest of the row means anything.
     * A tick logged at 30 Hz was computed with pid->dt = 0.01 s regardless, so
     * its d_term is inflated by the ratio -- the gains on that row are not the
     * gains in robot.conf. */
    r->loop_hz = g_telemetry_data.system.loop_hz;
    r->link_down = motor_hal_link_down();
    /* What the carrot is asking for. Without these two a log cannot say
     * whether the bot was tracking the planned move or running away from it. */
    r->carrot_lead = carrot_valid ? (carrot - (float)state.enc_pos) : 0.0f;
    r->carrot_vel = carrot_valid ? carrot_v : 0.0f;
    /* Both yaw rates. Plot them together: same shape and same sign means
     * yaw_gyro_scale is right, mirrored means flip its sign, and a constant
     * ratio is the factor to multiply it by. */
    r->yaw_gyro_rate = yaw_rate_gyro;
    r->yaw_enc_rate = yaw_rate_enc;
    /* And the raw gyro, unscaled. yaw_gyro_rate is psi_dot TIMES
     * yaw_gyro_scale, so while the scale is still 0 -- which is exactly the
     * state you are in when you need to calibrate it -- that column is all
     * zeros and the fit has nothing to work with. Logging psi_dot itself means
     * the scale can be fitted from any capture in which the robot was turned:
     * no special run, no temporary scale of 1. */
    r->yaw_psi_dot = state.psi_dot;
}


/* Config block ("# ..." lines): the settings a run was recorded under. */
static void looplog_write_config(FILE *f)
{
    fprintf(f, "# pit_gains: kp=%g ki=%g kd=%g\n",
            g_telemetry_data.pitch.kp, g_telemetry_data.pitch.ki, g_telemetry_data.pitch.kd);
    fprintf(f, "# yaw_gains: kp=%g ki=%g kd=%g\n",
            g_telemetry_data.yaw.kp, g_telemetry_data.yaw.ki, g_telemetry_data.yaw.kd);
    fprintf(f, "# pos_config: zone_a=%d zone_b=%d zone_c=%d\n",
            g_pos_config.zone_a, g_pos_config.zone_b, g_pos_config.zone_c);
    fprintf(f, "#   scale_a=%g scale_b=%g scale_c=%g scale_d=%g\n",
            g_pos_config.scale_a, g_pos_config.scale_b, g_pos_config.scale_c, g_pos_config.scale_d);
    fprintf(f, "#   vel_scale_stop=%g vel_scale_move=%g vel_scale_turning=%g\n",
            g_pos_config.vel_scale_stop, g_pos_config.vel_scale_move, g_pos_config.vel_scale_turning);
    /* These six used to be missing, and their absence cost a full tuning
     * session: a capture could not say which velocity source fed the damping
     * term, whether the damping filter was on, or what the integral was capped
     * at -- so a run made with changed settings was indistinguishable from one
     * where the change never reached the firmware. A log that cannot state the
     * configuration it was recorded under cannot be compared with another log. */
    fprintf(f, "#   vel_src=%d vel_damp_fc=%g vel_damp_max=%g\n",
            g_pos_config.vel_src, g_pos_config.vel_damp_fc, g_pos_config.vel_damp_max);
    fprintf(f, "#   pos_ki=%g pos_i_max=%g stopped_vel=%d\n",
            g_pos_config.pos_ki, g_pos_config.pos_i_max, g_pos_config.stopped_vel);
    fprintf(f, "#   max_correction=%g max_angle_rate=%g pos_deadband=%d back_to_spot=%d\n",
            g_pos_config.max_correction, g_pos_config.max_angle_rate,
            g_pos_config.pos_deadband, g_pos_config.back_to_spot);
    fprintf(f, "#   lead_max=%d return_rate=%g return_accel=%g\n",
            g_pos_config.lead_max, g_pos_config.return_rate, g_pos_config.return_accel);
    fprintf(f, "# yaw_gyro_scale=%g (0 = D from encoders)\n", g_yaw_gyro_scale);
    fprintf(f, "# theta_trim=%g oob_angle_deg=%g\n",
            state.theta_offset, g_oob_angle_deg);
    fprintf(f, "# motor: mode=%d qpps_max=%d claw_kp=%g claw_ki=%g claw_kd=%g\n",
            g_motor_config.mode, g_motor_config.qpps_max,
            g_motor_config.claw_kp, g_motor_config.claw_ki, g_motor_config.claw_kd);

}

static void looplog_write_columns(FILE *f)
{
    /* Same column names as the dashboard CSV export, on purpose. */
    fprintf(f, "t,pit_setpoint,pit_measurement,pit_error,pit_pTerm,pit_iTerm,pit_dTerm,"
               "pit_output,pos_encTarget,pos_encPos,pos_encError,pos_encVel,pos_encVelRaw,"
               "pos_encVelMid,pos_encVelLong,pos_posCorr,pos_velDamp,pos_posIterm,pos_thetaAdj,"
               "pos_activeScale,yaw_setpoint,yaw_measurement,yaw_error,yaw_pTerm,yaw_iTerm,"
               "yaw_dTerm,yaw_output,mot_leftDuty,mot_rightDuty,mot_dutyDiff,"
               "sys_loopHz,sys_linkDown,pos_carrotLead,pos_carrotVel,"
               "yaw_gyroRate,yaw_encRate,yaw_psiDot\n");

}

/* One CSV row. t is written as given (seconds). */
static int looplog_write_row(FILE *f, const looplog_row_t *r, double t)
{
    return fprintf(f,
                   "%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,"
                   "%d,%d,%d,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,"
                   "%.4f,%.4f,%.4f,%.1f,%d,%.2f,%.1f,%.3f,%.3f,%.3f\n",
                   t, r->pit_sp, r->pit_meas, r->pit_err, r->pit_p, r->pit_i, r->pit_d, r->pit_out,
                   r->enc_target, r->enc_pos, r->enc_err,
                   r->enc_vel, r->enc_vel_raw, r->enc_vel_mid, r->enc_vel_long,
                   r->pos_corr, r->vel_damp, r->pos_iterm, r->theta_adj, r->active_scale,
                   r->yaw_sp, r->yaw_meas, r->yaw_err, r->yaw_p, r->yaw_i, r->yaw_d, r->yaw_out,
                   r->left_duty, r->right_duty, r->right_duty - r->left_duty,
                   r->loop_hz, r->link_down, r->carrot_lead, r->carrot_vel,
                   r->yaw_gyro_rate, r->yaw_enc_rate, r->yaw_psi_dot);
}

/* ── Always-on 100 Hz ring ─────────────────────────────────────────────────
 * Every control tick is appended to a file in /dev/shm (RAM -- never the SD
 * card or eMMC, so no wear and nothing for a power cut to corrupt). Two
 * segments of RING_SEG_BYTES each: when the current one fills, the older one
 * is truncated and becomes current. So the last 5-10 MB of history (roughly
 * 4-7 minutes at 100 Hz) is always on the bot, and whatever just happened --
 * a takeoff, a jump -- is already recorded before anyone thinks to press a
 * button. serve_web.py joins the two into /bbot_ring.csv.
 *
 * Cost per tick: one formatted row into a stdio buffer; a write() to tmpfs
 * every RING_FLUSH_ROWS rows. No thread, no burst.
 *
 * t is ABSOLUTE seconds since boot (same clock as the telemetry "timestamp"),
 * so the two segments line up and rows can be matched to a dashboard CSV.
 * Any config change is written in-stream as "# CHANGE t=..." followed by the
 * full config block, so a knob turned mid-run is visible where it happened. */
#define RING_DIR "/dev/shm"
#define RING_SEG_BYTES (5L * 1024 * 1024)
#define RING_FLUSH_ROWS 10

typedef struct
{
    float pk, pi, pd, yk, yi, yd, trim;
    pos_config_t pos;
    motor_config_t mot;
} ring_cfg_t;

static void ring_cfg_now(ring_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->pk = g_telemetry_data.pitch.kp;
    c->pi = g_telemetry_data.pitch.ki;
    c->pd = g_telemetry_data.pitch.kd;
    c->yk = g_telemetry_data.yaw.kp;
    c->yi = g_telemetry_data.yaw.ki;
    c->yd = g_telemetry_data.yaw.kd;
    c->trim = state.theta_offset;
    memcpy(&c->pos, &g_pos_config, sizeof(c->pos));
    memcpy(&c->mot, &g_motor_config, sizeof(c->mot));
}

static void ring_tick(void)
{
    static FILE *f = NULL;
    static int seg = 0;
    static long bytes = 0;
    static int rows = 0;
    static int dead = 0;
    static ring_cfg_t last;

    if (dead)
        return;

    uint64_t now = rc_nanos_since_boot() / 1000;
    ring_cfg_t cur;
    ring_cfg_now(&cur);

    if (!f || bytes >= RING_SEG_BYTES)
    {
        if (f)
        {
            fclose(f);
            seg ^= 1;
        }
        char path[64];
        snprintf(path, sizeof path, RING_DIR "/bbot_ring_%d.csv", seg);
        f = fopen(path, "w");
        if (!f)
        {
            LOG_WARN("ring: cannot open %s -- 100 Hz ring disabled", path);
            dead = 1;
            return;
        }
        bytes = fprintf(f, "# bbot 100 Hz ring segment %d (t = seconds since boot)\n"
                           "# rate_hz=%d mode=ring\n", seg, SAMPLE_RATE_HZ);
        looplog_write_config(f);
        looplog_write_columns(f);
        bytes += 2048; /* header; exact size does not matter */
        last = cur;
    }
    else if (memcmp(&cur, &last, sizeof cur) != 0)
    {
        bytes += fprintf(f, "# CHANGE t=%.3f config changed; now:\n", (double)now / 1e6);
        looplog_write_config(f);
        bytes += 1024;
        last = cur;
    }

    looplog_row_t r;
    looplog_fill_row(&r, now);
    int n = looplog_write_row(f, &r, (double)now / 1e6);
    if (n > 0)
        bytes += n;
    if (++rows >= RING_FLUSH_ROWS)
    {
        fflush(f);
        rows = 0;
    }
}


/**
 * @brief Initialize robot hardware and control system
 */
int robot_init(void)
{
    if (rc_kill_existing_process(2.0) < -2)
        return -1;

    if (rc_enable_signal_handler() == -1)
    {
        fprintf(stderr, "ERROR: Failed to start signal handler\n");
        return -1;
    }

    rc_adc_init(); /* stub — battery reading handled by telemetry */

    // PIDs
    pid_init(&pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD, DT);
    pid_init(&yaw_pid, YAW_KP, YAW_KI, YAW_KD, DT);
    // pitch_pid.integrator_max = 4.0f;

    // Initialise position controller config from compile-time defaults.
    // These are overwritten by pid_config_load_or_default() in main(),
    // and can be updated at runtime via IPC set_pos_config.
    g_pos_config.zone_a = POS_ZONE_A_DEFAULT;
    g_pos_config.zone_b = POS_ZONE_B_DEFAULT;
    g_pos_config.zone_c = POS_ZONE_C_DEFAULT;
    g_pos_config.scale_a = POS_SCALE_A_DEFAULT;
    g_pos_config.scale_b = POS_SCALE_B_DEFAULT;
    g_pos_config.scale_c = POS_SCALE_C_DEFAULT;
    g_pos_config.scale_d = POS_SCALE_D_DEFAULT;
    g_pos_config.vel_scale_stop = POS_VEL_SCALE_STOP_DEFAULT;
    g_pos_config.vel_scale_move = POS_VEL_SCALE_MOVE_DEFAULT;
    g_pos_config.vel_scale_turning = POS_VEL_SCALE_TURNING_DEFAULT;
    g_pos_config.stopped_vel = POS_STOPPED_VEL_DEFAULT;
    g_pos_config.max_correction = POS_MAX_CORRECTION_DEFAULT;
    g_pos_config.max_angle_rate = POS_MAX_ANGLE_RATE_DEFAULT;
    g_pos_config.back_to_spot = POS_BACK_TO_SPOT_DEFAULT;
    g_pos_config.drive_mode = POS_DRIVE_MODE_DEFAULT;
    g_pos_config.drive_rate = POS_DRIVE_RATE_DEFAULT;
    g_pos_config.runaway_limit = POS_RUNAWAY_LIMIT_DEFAULT;
    g_pos_config.lead_max = POS_LEAD_MAX_DEFAULT;
    g_pos_config.return_rate = POS_RETURN_RATE_DEFAULT;
    g_pos_config.return_accel = POS_RETURN_ACCEL_DEFAULT;

    g_motor_config.mode = MOTOR_HAL_MODE_DEFAULT;
    g_motor_config.qpps_max = MOTOR_QPPS_MAX_DEFAULT;
    g_motor_config.accel_qpps = MOTOR_ACCEL_QPPS_DEFAULT;
    g_motor_config.max_amps = MOTOR_MAX_AMPS_DEFAULT;
    g_motor_config.pol_l = 1.0f;
    g_motor_config.pol_r = 1.0f;
    LOG_INFO("PID Controllers:");
    LOG_INFO("  D1_balance:  Kp=%.3f Ki=%.3f Kd=%.3f", PITCH_KP, PITCH_KI, PITCH_KD);
    LOG_INFO("  D3_steering: Kp=%.3f Ki=%.3f Kd=%.3f", YAW_KP, YAW_KI, YAW_KD);

    // IMU config
    // IMU offsets arrive with everything else via robot_config_apply() in
    // main(), immediately after this function returns.

    // IMU / DMP
    LOG_INFO("Initializing IMU...");
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
    mpu_config.dmp_fetch_accel_gyro = 1;

    if (rc_mpu_initialize_dmp(&mpu_data, mpu_config) < 0)
    {
        fprintf(stderr, "ERROR: IMU initialization failed\n");
        return -1;
    }
    rc_mpu_set_dmp_callback(&imu_interrupt);

    // Motor HAL initialized by main() before robot_init()
    // Estop: init GPIO but leave deasserted — roboclaw_reset.py brings unit up clean.
    // Estop is asserted on disarm/fall and deasserted on arm via robot_run transitions.
    roboclaw_estop_init();

    // LEDs
    rc_led_set(RC_LED_GREEN, 0);
    rc_led_set(RC_LED_RED, 0);

    // Initial state
    state.mode = MODE_BALANCE;
    state.armed = 0;
    state.theta_ref = 0.0f;
    state.theta_offset = 0.0f;
    state.yaw = 0.0f;
    state.pos_correction = 0.0f;
    state.pos_vel_damp = 0.0f;
    state.pos_output = 0.0f;
    state.pos_scale = 0.0f;

    rc_make_pid_file();
    rc_set_state(RUNNING);

    LOG_INFO("Robot initialized successfully");
    return 0;
}

// Reads 3S LiPo via BBB AIN1 (68k/10k divider, 7.8x)
// or falls back to RoboClaw main battery voltage.
static int read_battery_voltage(float *volts)
{
    // Try BBB ADC first (AIN channel 1, sysfs IIO)
    int fd = open("/sys/bus/iio/devices/iio:device0/in_voltage1_raw", O_RDONLY);
    if (fd >= 0)
    {
        char buf[16] = {0};
        int n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n > 0)
        {
            int raw = atoi(buf);
            // BBB AIN: 12-bit, 0–1.8V ref. Divider ratio 7.8x (68k/10k).
            *volts = (raw / 4095.0f) * 1.8f * 7.8f;
            if (*volts > 5.0f) // sanity: must look like a real battery
                return 0;
        }
    }

    // Fall back to RoboClaw
    return motor_hal_read_voltage(volts);
}

/**
 * @brief Main control loop (~100 Hz)
 */
/* The motion gate (see motor_hal.h). Every condition is read fresh at the
 * moment a drive command is about to be sent. All must hold; any one failing
 * turns the command into a coast. This duplicates the ISR's own actuation
 * check on purpose: that check decides what to ASK for, this one decides what
 * the RoboClaw is allowed to RECEIVE, whoever asked. */
static int robot_motion_gate(const char **why)
{
    if (!state.armed)            { *why = "not armed"; return 0; }
    if (!state.trying)           { *why = "not trying"; return 0; }
    if (state.estop_latched)     { *why = "e-stop latched"; return 0; }
    if (g_imu_samples < 50u)     { *why = "IMU has not produced 50 samples"; return 0; }
    if (g_imu_stale)             { *why = "IMU stale"; return 0; }
    float eff = fabsf(state.theta - state.theta_offset);
    if (!(eff <= g_oob_angle_deg)) /* also catches NaN */
    {
        static char buf[96];
        snprintf(buf, sizeof buf, "out of bounds (theta=%.2f eff=%.2f limit=%.1f)",
                 state.theta, eff, (double)g_oob_angle_deg);
        *why = buf;
        return 0;
    }
    return 1;
}

void robot_run(void)
{
    motor_hal_set_motion_gate(robot_motion_gate);

    uint64_t loop_counter = 0;
    uint64_t last_loop_us = 0;

    while (rc_get_state() != EXITING)
    {

        // ── Loop rate measurement ─────────────────────────────────────────
        uint64_t now_us = rc_nanos_since_boot() / 1000;
        if (last_loop_us > 0)
        {
            float dt = (now_us - last_loop_us) / 1000000.0f;
            if (dt > 0.0001f)
                g_telemetry_data.system.loop_hz = 1.0f / dt;
        }
        last_loop_us = now_us;

        // ── Battery voltage ───────────────────────────────────────────────
        static uint64_t last_batt_us = 0;
        if (now_us - last_batt_us >= 1000000ULL)
        {
            float volts = 0.0f;
            if (read_battery_voltage(&volts) == 0)
                g_telemetry_data.system.battery_voltage = volts;
            last_batt_us = now_us;
        }

        // ── Arm at boot ──────────────────────────────────────────────────
        // Stand inert on the kickstand however long it takes, then start
        // balancing when picked up -- no network, no dashboard, no button.
        //
        // The bot parks at ~19 deg, past the OOB limit, so "out of bounds" is
        // the normal resting state and is not a fault.
        //
        // ORDER MATTERS. The bot must SEE the kickstand before it may arm.
        // On this machine a valid sensor cannot read in-bounds at boot -- the
        // stand holds it at 19 deg. So a theta that sits near zero from boot
        // is a broken sensor, not a ready robot, and arming on it drives the
        // wheels with no working feedback until someone pulls the battery.
        // Requiring an out-of-bounds reading first makes that failure inert:
        // a frozen theta never sets the latch, so it never arms.
#define ARM_MIN_SAMPLES 50u     /* samples before theta is believed at all */
#define ARM_STEADY_US 250000ULL /* held inside the window this long */
/* Arm INSIDE this window, which is deliberately tighter than g_oob_angle_deg:
 * the OOB limit is where motors get cut, not a sane place to start balancing.
 * The rate limit stops it arming while swinging through on the way elsewhere. */
#define ARM_ANGLE_DEG 3.0f
#define ARM_RATE_DPS 15.0f
/* The fused DMP theta takes seconds to converge after boot, slewing from a
 * wrong start (75 deg logged 2026-09-19) toward the real kickstand angle while
 * the robot sits perfectly still. The rate gate above uses the RAW GYRO, which
 * correctly reads ~0 the whole time, so it cannot see the slew. A slew toward
 * -19 passes through 0, and at ~7 deg/s it sits inside the 3 deg window for
 * well over 250 ms -- so the old code armed ON THE KICKSTAND ("armed theta=0.28
 * rate=-0.4") and drove the wheels until the estimate crossed the OOB limit.
 * Fix: the estimate itself must be still. theta's own span over the window,
 * not the gyro, decides. */
#define ARM_THETA_SPAN_DEG 1.0f      /* max theta span while "still" */
#define KICKSTAND_SETTLE_US 1000000ULL /* kickstand must read steady this long */
        static int boot_arm_done = 0; /* once per run: a deliberate
                                         disarm must stay disarmed */
        static uint64_t in_range_since_us = 0;
        static int waiting_logged = 0;
        static uint64_t ks_since_us = 0; /* kickstand steady-window start */
        static float ks_min = 0.0f, ks_max = 0.0f;
        static float hold_min = 0.0f, hold_max = 0.0f;
        static int slew_logged = 0;
        static int seen_kickstand = 0; /* sensor proved it can see 19 deg */
        static int kickstand_logged = 0;
        if (!boot_arm_done && g_arm_at_boot)
        {
            float eff = fabsf(state.theta - state.theta_offset);

            /* Kickstand check. Until the sensor has shown us an out-of-bounds
             * attitude, we have no evidence it is reading the robot at all. */
            if (g_imu_samples >= ARM_MIN_SAMPLES && !seen_kickstand)
            {
                if (eff > g_oob_angle_deg)
                {
                    /* Kickstand = out of bounds AND the estimate has stopped
                     * moving. A converging estimate is still slewing, so it
                     * keeps restarting this window until it settles. */
                    if (ks_since_us == 0)
                    {
                        ks_min = ks_max = state.theta;
                        ks_since_us = now_us;
                    }
                    if (state.theta < ks_min) ks_min = state.theta;
                    if (state.theta > ks_max) ks_max = state.theta;
                    if (ks_max - ks_min > ARM_THETA_SPAN_DEG)
                    {
                        ks_min = ks_max = state.theta; /* still moving: restart */
                        ks_since_us = now_us;
                    }
                    if (now_us - ks_since_us >= KICKSTAND_SETTLE_US)
                    {
                        seen_kickstand = 1;
                        LOG_WARN("arm_at_boot: kickstand seen, estimate settled "
                                 "(theta=%.2f eff=%.2f, span %.2f deg over %llums). "
                                 "Arming enabled.",
                                 state.theta, eff, ks_max - ks_min,
                                 (unsigned long long)(KICKSTAND_SETTLE_US / 1000));
                    }
                }
                else
                {
                    ks_since_us = 0;
                    if (!kickstand_logged)
                    {
                        kickstand_logged = 1;
                        LOG_WARN("arm_at_boot: BLOCKED -- theta=%.2f eff=%.2f reads "
                                 "in-bounds at boot, but the kickstand holds this bot "
                                 "past %.1f deg. Sensor is not trustworthy; will not arm.",
                                 state.theta, eff, (double)g_oob_angle_deg);
                    }
                }
            }

            if (g_imu_samples < ARM_MIN_SAMPLES || !seen_kickstand)
            {
                /* No believable measurement yet, or the kickstand was never
                 * seen. Either way nothing may act on state.theta. */
                in_range_since_us = 0;
            }
            else if (state.estop_latched || state.armed)
            {
                in_range_since_us = 0;
            }
            else if (eff > ARM_ANGLE_DEG || fabsf(state.theta_dot) > ARM_RATE_DPS)
            {
                in_range_since_us = 0;
                if (!waiting_logged)
                {
                    waiting_logged = 1;
                    LOG_WARN("arm_at_boot: ready, waiting to be stood up "
                             "(theta=%.2f eff=%.2f, needs |eff|<%.1f and "
                             "|rate|<%.0f deg/s). Will arm by itself once "
                             "upright and still.",
                             state.theta, eff, (double)ARM_ANGLE_DEG,
                             (double)ARM_RATE_DPS);
                }
            }
            else
            {
                if (in_range_since_us == 0)
                {
                    in_range_since_us = now_us;
                    hold_min = hold_max = state.theta;
                }
                if (state.theta < hold_min) hold_min = state.theta;
                if (state.theta > hold_max) hold_max = state.theta;
                if (hold_max - hold_min > ARM_THETA_SPAN_DEG)
                {
                    /* Gyro says still, estimate says moving: the estimate is
                     * still converging. Not a robot being held upright. */
                    if (!slew_logged)
                    {
                        slew_logged = 1;
                        LOG_WARN("arm_at_boot: theta slewing %.2f deg while gyro reads "
                                 "%.1f deg/s -- estimate not converged, NOT arming",
                                 hold_max - hold_min, state.theta_dot);
                    }
                    in_range_since_us = now_us;
                    hold_min = hold_max = state.theta;
                }

                if (now_us - in_range_since_us >= ARM_STEADY_US)
                {
                    boot_arm_done = 1;
                    state.armed = 1;
                    state.trying = 1;
                    motor_hal_standby(0);
                    rc_led_set(RC_LED_GREEN, 1);
                    /* WARN, not INFO: the service runs --quiet, so INFO is
                     * dropped. Motors starting with nobody pressing anything
                     * earns a line at any verbosity. */
                    LOG_WARN("arm_at_boot: armed (theta=%.2f theta_offset=%.2f "
                             "eff=%.2f rate=%.1f, held inside %.1f deg for "
                             "%llums, %u samples)",
                             state.theta, state.theta_offset, eff,
                             state.theta_dot, (double)ARM_ANGLE_DEG,
                             (unsigned long long)(ARM_STEADY_US / 1000),
                             (unsigned)g_imu_samples);
                }
            }
        }

        // ── Disarm handling ───────────────────────────────────────────────
        static int prev_armed = 0;
        if (prev_armed && !state.armed)
        {
            // Disarm: stop motors, reset PIDs. No estop — that's only for E-STOP button.
            pending_left_duty = 0.0f;
            pending_right_duty = 0.0f;
            motor_output_ready = 0;
            motor_hal_coast(); /* off, not "hold zero speed" */
            pid_reset(&pitch_pid);
            pid_reset(&yaw_pid);
            last_left_duty = 0.0f;
            last_right_duty = 0.0f;
            state.enc_pos_target = state.enc_pos;
            pos_integ = 0.0f; /* target moved — accumulated error is stale */
            carrot_valid = 0; carrot_v = 0.0f; carrot_arrived = 0; /* target re-snapped: carrot restarts at the wheels */
        }
        if (!prev_armed && state.armed)
        {
            state.enc_pos_target = state.enc_pos;
            pos_integ = 0.0f; /* target moved — accumulated error is stale */
            carrot_valid = 0; carrot_v = 0.0f; carrot_arrived = 0; /* target re-snapped: carrot restarts at the wheels */
            state.theta_ref = 0.0f;
            // trying is persistent now -- tracks armed directly, no angle gate.
            // The ISR's own eff_angle>g_oob_angle_deg check (imu_interrupt, above
            // in this file) is what actually protects a bad arming angle; it
            // just means trying=1 sits inert until in range.
            state.trying = 1;
        }
        prev_armed = state.armed;

        // ── Motor write ───────────────────────────────────────────────────
        if (motor_output_ready)
        {
            float l = pending_left_duty;
            float r = pending_right_duty;
            motor_output_ready = 0;
            motor_hal_set_both(l, r);
            last_left_duty = l;
            last_right_duty = r;
        }

        // ── Encoder read ──────────────────────────────────────────────────
        int left_ticks = motor_hal_encoder_read(MOTOR_LEFT);
        int right_ticks = motor_hal_encoder_read(MOTOR_RIGHT);
        state.enc_left = left_ticks;
        state.enc_right = right_ticks;
        state.phi_left = left_ticks * (360.0f / ENCODER_TICKS_PER_REV);
        state.phi_right = right_ticks * (360.0f / ENCODER_TICKS_PER_REV);
        state.pos = (state.phi_left + state.phi_right) / 2.0f;

        // ── Encoder position + velocity ───────────────────────────────────
        state.enc_pos = left_ticks + right_ticks;
        {
            /* Encoder velocity: least-squares slope of enc_pos over the last
             * POS_VEL_WINDOW control ticks, recomputed EVERY tick.
             *
             * This replaces reading the RoboClaw's speed registers every
             * POS_VEL_PERIOD_MS, which produced a 25 Hz staircase in a 100 Hz
             * loop. See POS_VEL_WINDOW in balance_bot.h for why that mattered:
             * vel_damp is applied after the position rate limiter, so every
             * staircase step hit theta_ref instantly, and the damping needed to
             * kill the loop's ~0.55 Hz ring could not be turned up without
             * the steps themselves becoming the disturbance.
             *
             * Units are unchanged -- ticks/100ms -- so existing
             * vel_scale_stop / vel_scale_move values keep their meaning.
             *
             * A plain tick-to-tick difference was tried before and rejected
             * because it truncates to zero: at 100 Hz consecutive reads often
             * differ by 0 ticks. A least-squares slope over a window does not
             * have that problem -- it fits through the quantisation rather
             * than differencing across it.
             */
            /* Velocity candidates, all computed every tick from the same
             * position history, so one recorded run compares them directly.
             *
             * History so far, for whoever reads this next:
             *   - The RoboClaw speed registers (the incumbent) are read every
             *     POS_VEL_PERIOD_MS and held. That was assumed to be a crude
             *     staircase. It is not -- measured against a 6-tick
             *     least-squares slope it is 1.79x SMOOTHER, because the
             *     RoboClaw filters internally. The staircase theory was wrong.
             *   - What the RoboClaw actually costs is LAG: measured at 50-100 ms
             *     behind the least-squares estimate. That, not step kicks, is
             *     what makes high damping unstable -- vel_scale_stop=3.5 rang
             *     because of phase lag, not because of discontinuities.
             *   - So a longer window is the interesting region: still less lag
             *     than the RoboClaw, but smoother than w=6. Hence three
             *     candidates rather than one guess.
             *
             * Delete the extra candidates once a window has been chosen. */
            static int32_t vel_hist[POS_VEL_WIN_MAX];
            static int vel_head = 0; /* next write slot == oldest sample */
            static int vel_fill = 0;

            /* ticks-per-control-tick -> ticks/100ms */
            const float VEL_UNIT = 0.1f * (float)SAMPLE_RATE_HZ;

            static uint64_t last_vel_us = 0;
            static float vel_claw = 0.0f;
            uint64_t now_us = rc_nanos_since_boot() / 1000;

            if (state.enc_vel_reset)
            {
                vel_fill = 0;
                vel_head = 0;
                vel_claw = 0.0f;
                state.enc_velocity = 0.0f;
                state.enc_velocity_raw = 0.0f;
                state.enc_vel_lsq_mid = 0.0f;
                state.enc_vel_lsq_long = 0.0f;
                last_vel_us = now_us;
                state.enc_vel_reset = 0;
            }
            else
            {
                vel_hist[vel_head] = state.enc_pos;
                vel_head = (vel_head + 1) % POS_VEL_WIN_MAX;
                if (vel_fill < POS_VEL_WIN_MAX)
                    vel_fill++;

                /* Least-squares slope over the newest `win` samples.
                 * slope = sum((k - kbar) * y_k) / (N(N^2-1)/12), oldest first. */
                float lsq[3] = {0.0f, 0.0f, 0.0f};
                const int wins[3] = {POS_VEL_WIN_SHORT, POS_VEL_WIN_MID, POS_VEL_WIN_LONG};
                for (int c = 0; c < 3; c++)
                {
                    int win = wins[c];
                    if (vel_fill < win)
                        continue;
                    const float kbar = (win - 1) * 0.5f;
                    const float den = (float)win * ((float)win * win - 1.0f) / 12.0f;
                    float num = 0.0f;
                    for (int k = 0; k < win; k++)
                    {
                        /* newest `win` samples: walk back from the newest */
                        int idx = (vel_head - win + k + 2 * POS_VEL_WIN_MAX) % POS_VEL_WIN_MAX;
                        num += ((float)k - kbar) * (float)vel_hist[idx];
                    }
                    lsq[c] = (num / den) * VEL_UNIT;
                }

                if ((now_us - last_vel_us) >= (POS_VEL_PERIOD_MS * 1000ULL))
                {
                    int32_t m1 = 0, m2 = 0;
                    if (motor_hal_read_encoder_speeds(&m1, &m2) == 0)
                        vel_claw = (float)(m1 + m2) / 10.0f;
                    last_vel_us = now_us;
                }

                /* Only one of these controls. The rest are evidence.
                 * Runtime-selected so velocity sources can be A/B'd against
                 * each other in one session instead of one per rebuild. */
                {
                    int src = g_pos_config.vel_src;
                    if (src < 0 || src > 3)
                        src = 0;
                    state.enc_velocity = (src == 0) ? vel_claw : lsq[src - 1];
                    state.enc_velocity_raw = (src == 0) ? lsq[0] : vel_claw;
                }
                state.enc_vel_lsq_mid = lsq[1];
                state.enc_vel_lsq_long = lsq[2];
            }
        }

        /* The MODE_EXT_INPUT branch that lived here (UART "PKT,x,y,conf")
         * was removed 2026-09-21: nothing ever sent it, it wrote a raw lean of
         * up to MAX_THETA_REF straight into theta_ref, and it assigned the
         * heading target instead of integrating a turn rate, so it could not
         * have turned the robot anyway. External drive now comes from the Pi
         * through pi_drive.c and the same path as the stick, below. */

        xbox_update();
        sbus_update();

        /* ── Drive input: SBUS stick, else the Pi, else nothing ────────────
         * Balancing never depends on any of this; it only moves the targets
         * the balance loop holds. One input per tick, by priority:
         *   1. the SBUS stick, when it is off centre -- the operator wins;
         *   2. a fresh Pi command, when armed, the Pi-drive gate is open and
         *      the RC kill switch (if a transmitter is on) is at RUN;
         *   3. a centred stick, when a transmitter is connected;
         *   4. nothing.
         * Both reach the controller through the same code below, so the Pi
         * gets drive_mode, drive_rate/drive_scale and turn_rate exactly as
         * the stick does. The kick-to-stand path still reads SBUS only. */
        float in_drive = 0.0f, in_turn = 0.0f;
        int have_input = 0;
        {
            const int sbus_ok = sbus_is_connected() && state.armed;
            const float s_drive = sbus_ok ? sbus_get_drive() : 0.0f;
            const float s_turn = sbus_ok ? sbus_get_turn() : 0.0f;
            float px, py;
            const int pi_fresh = pi_drive_get(&px, &py);
            if (!state.armed)
                pi_drive_set_gate(0);     /* disarm, e-stop, boot: gate shuts */
            const pi_drive_state_t pi = pi_drive_decide(
                state.armed, pi_drive_gate(), sbus_is_connected(),
                sbus_get_kill(), s_drive, s_turn, pi_fresh);
            pi_drive_set_state(pi);

            if (pi == PI_DRIVE_APPLYING)
            {
                in_drive = py;
                in_turn = px;
                have_input = 1;
            }
            else if (sbus_ok)
            {
                in_drive = s_drive;
                in_turn = s_turn;
                have_input = 1;
            }
        }

        // ── SBUS drive commands (no arming via SBUS — IPC only) ───────────
        // stick_input tracks operator drive intent only — never touched by D2.
        // This is what raw_stick_ref must read, not state.theta_ref which
        // accumulates D2 correction across loops.
        static float stick_input = 0.0f;
        // Normalised drive command, deadband already applied by input_sbus.c.
        // Kept alongside stick_input because "is the operator driving?" must be
        // asked of the STICK, not of the lean angle it happens to produce --
        // see the stick_centered test below.
        static float stick_norm = 0.0f;

        /* Heading target, in the same units as phi_diff: degrees of differential
         * wheel rotation. The steering loop is a POSITION loop on heading, so the
         * stick must command a RATE that we integrate. Assigning the stick
         * straight to the setpoint (as this did) asks for a fixed heading offset
         * of at most MAX_YAW_CMD = 1 degree of wheel differential -- roughly
         * 1.3 mm of differential travel, and completely invisible. That is why
         * the bot would not turn no matter how the channels were mapped. */

        /* Fractional carry for target-mode drive: at 100 Hz a full-stick
         * advance is only a few ticks per loop, so truncating every tick would
         * throw most of the command away. */
        static float drive_accum = 0.0f;

        if (have_input)
        {
            stick_norm = in_drive;

            if (g_pos_config.drive_mode == DRIVE_MODE_TARGET)
            {
                /* Stick commands VELOCITY: advance where we want to be and let
                 * position hold work out the lean. Centring the stick stops the
                 * target moving, the bot coasts past it, and the growing error
                 * produces the braking lean on its own -- no reverse stick, no
                 * timing judgement. theta_ref is an OUTPUT in this mode, so the
                 * stick must not touch it. */
                drive_accum += stick_norm * g_pos_config.drive_rate * DT;
                int32_t whole = (int32_t)drive_accum;
                if (whole != 0)
                {
                    state.enc_pos_target += whole;
                    drive_accum -= (float)whole;
                }
                stick_input = 0.0f;
            }
            else
            {
                /* Stick commands ACCELERATION (legacy). */
                stick_input = stick_norm * MAX_THETA_REF;
                state.theta_ref = stick_input;
            }

            /* Hold the stick over and the bot keeps turning; release and it
             * holds the heading it reached. */
            yaw_target += in_turn * g_sbus_config.turn_rate * DT;
            state.yaw = yaw_target;
        }
        else
        {
            drive_accum = 0.0f;
        }

        if (g_pos_config.drive_mode == DRIVE_MODE_TARGET)
        {
            /* Anti-windup. A blocked or lifted wheel lets the target run away
             * from where the bot can actually get to; the debt then discharges
             * violently the moment traction returns. Kept well inside zone_c so
             * the loose-hold abandon branch is not triggered by it. */
            int32_t lim = g_pos_config.runaway_limit;
            if (lim > 0)
            {
                int32_t lead = state.enc_pos_target - state.enc_pos;
                if (lead > lim)
                    state.enc_pos_target = state.enc_pos + lim;
                else if (lead < -lim)
                    state.enc_pos_target = state.enc_pos - lim;
            }
        }
        else if (!have_input)
        {
            /* No stick and no Pi. Was !sbus_is_connected(): with the Pi as a
             * second source, a Pi-driven tick with the transmitter off must not
             * have its input zeroed here before raw_stick_ref reads it. */
            stick_input = 0.0f;
            stick_norm = 0.0f;
        }

        /* Disarmed: forget the accumulated heading so re-arming does not
         * immediately spin toward a target set minutes ago.
         *
         * The !sbus_is_connected() half of this test was removed 2026-09-20.
         * This robot is normally driven with the transmitter OFF, so that
         * clause was true on every tick and the heading target was reassigned
         * to the current heading 100 times a second -- the steering loop had
         * no fixed heading to hold and a twist was adopted as the new home
         * before it could ever be corrected. Measured on
         * bbot_ring_1789950092231: median |yaw_setpoint - phi_diff| = 0.000
         * and median |yaw_error| = 0.000 across 5013 upright ticks, i.e. the
         * loop was regulating against a target that WAS the measurement.
         * Losing the link is a reason to stop taking new stick commands, which
         * the drive path already handles; it is not a reason to give up the
         * heading the robot is standing on. */
        if (!state.armed)
        {
            yaw_target = (state.phi_right - state.phi_left) / 2.0f;
            state.yaw = yaw_target;
        }

        // Capture stick input before D2 adds its correction.
        // theta_ref may have accumulated D2 correction from previous loops,
        // so raw_stick_ref must reflect only the operator's input.
        float raw_stick_ref = stick_input;
        float raw_stick_norm = stick_norm;

        /* D3 steering latch REMOVED 2026-09-19. It predates the heading target
         * and treated state.yaw as a -1..1 stick command: |state.yaw| >= 0.05
         * meant "turning", so any real heading armed it; the turndown walked the
         * target toward 0 with wheel speed; then it latched (phi_left -
         * phi_right)/2 -- the opposite sign of the yaw PID's measurement -- so
         * the target became minus the heading, pinned by the MAX_YAW_LEAD clamp
         * at exactly heading + 90. Telemetry 2026-09-19 20:37: setpoint -102.281,
         * measurement -192.281, error 90.000 with the remote off. Heading hold
         * is yaw_target (SBUS block above) plus the rebase on OOB recovery. */

        // ── Hard OOB cutoff (edge-triggered) ───────────────────────────────
        // trying is now persistent — it tracks armed directly (set together at
        // ARM/DISARM/E-STOP) and is no longer auto-cleared by angle. Ryan asked
        // to ungate the control loop and stop the angle hysteresis from
        // requiring a trip back through <10 deg before it would try again.
        //
        // The actual safety backstop does not depend on trying at all: while
        // eff_angle stays above 15 deg, motors are forced to 0 and the RoboClaw
        // held in standby unconditionally, every tick, regardless of armed or
        // trying. was_oob only tracks the edge so the reset bookkeeping below
        // (stale position target, stale integrators) and the recovery pid_reset
        // each run once per excursion instead of every tick.
        static int was_oob = 0;
        float eff_angle = fabsf(state.theta - state.theta_offset);
        if (eff_angle > g_oob_angle_deg)
        {
            /* The one exception to the cutoff: a deliberate, bounded, operator
             * -held kick to stand the robot up off its kickstand. Everything
             * that makes it safe is inside kick_direction(); if it returns 0
             * -- which is the case on every tick nobody is asking -- this
             * falls straight through to the coast below, unchanged. */
            const int kick = kick_direction(eff_angle);

            if (kick != 0)
            {
                /* Both wheels the same sign: drive the contact patch back
                 * under the centre of mass. Through set_both, so the motion
                 * gate still gets its say. */
                const float duty = (float)kick * g_kick_duty;
                motor_hal_set_both(duty, duty);
                /* Report what was actually written, same as the coast path
                 * below does, so the dashboard shows the kick instead of a
                 * frozen pre-OOB value. */
                last_left_duty = duty;
                last_right_duty = duty;
            }
            else
            {
                // OOB: always stop motors, every tick, regardless of was_oob.
                /* motor_hal_coast(), not set_both(0,0): in velocity mode a zero
                 * there is MIXEDSPEED asking the RoboClaw's velocity PID to HOLD
                 * zero, which is an active loop that winds up against stiction and
                 * pushes. The safety cutoff has to remove power, not regulate to
                 * zero. One write per tick now instead of two. */
                motor_hal_coast();
            /* Report what was actually written. last_*_duty is what telemetry
             * publishes, and it is otherwise only assigned on the normal motor
             * write and the disarm path -- so across an OOB excursion the
             * dashboard kept showing the duty from the instant it went out of
             * bounds, frozen, while the motors were dead. A 2026-09-13 push run
             * read 0.534 duty RMS during the cut and 0.277 while actually
             * driving, which inverts the truth and makes any log containing an
             * excursion unanalysable. */
            last_left_duty = 0.0f;
            last_right_duty = 0.0f;
            if (!was_oob)
            {
                was_oob = 1;
                rc_led_set(RC_LED_GREEN, 0);

                /* Zero the encoders outright, not just the target.
                 *
                 * Setting enc_pos_target = enc_pos makes the ERROR zero, which
                 * is what the controller reads, but enc_pos itself keeps
                 * whatever it accumulated across every fall of the session --
                 * -203 to +121 ticks in the 2026-09-13 capture, wandering
                 * further with each excursion. Once the bot is on its side and
                 * the wheels have been dragged across the floor, that number
                 * describes nothing. "Back to spot" then means back to a spot
                 * chosen by the last tumble.
                 *
                 * An excursion is the natural place to redefine the origin:
                 * wherever it ends up is the new home. Edge-triggered, so this
                 * is one serial write per excursion, not per tick, and it runs
                 * in the main loop where blocking I/O is already the norm
                 * (motor_hal_set_both above does the same).
                 *
                 * Same bookkeeping as the zero_encoders IPC command -- the
                 * hardware counters and the software's idea of them have to
                 * move together or enc_velocity reads as a huge step on the
                 * next tick. */
                /* REVERTED 2026-09-13: motor_hal_encoder_reset_all() was here.
                 *
                 * It fires on the OOB EDGE, and the bot parks at ~83 deg, so
                 * the edge trips within the first tick of every startup --
                 * meaning the first thing the service did was reset the
                 * encoder counters underneath a RoboClaw running in velocity
                 * mode (motor mode=1). Its internal loop sees the count jump
                 * as a velocity discontinuity and drives on it. Result: motors
                 * pushing from the moment the service comes up, with nothing
                 * in balance_bot commanding them, and not stopping.
                 *
                 * Zeroing the encoders is still the right idea, but it cannot
                 * be done blind while the drive is live. It needs to happen
                 * with the RoboClaw in standby AND with its velocity target
                 * re-commanded to 0 afterwards, or not at all. */
                state.enc_pos_target = state.enc_pos;
                pos_integ = 0.0f; /* target moved — accumulated error is stale */
                carrot_valid = 0; carrot_v = 0.0f; carrot_arrived = 0; /* target re-snapped: carrot restarts at the wheels */
                state.enc_velocity = 0;
                state.enc_vel_reset = 1;
                state.theta_ref = 0.0f;
                if (state.armed)
                    LOG_WARN("OOB — motors cut (eff=%.1f deg), staying armed", eff_angle);
                }
            }
        }
        else if (was_oob)
        {
            was_oob = 0;

            // Position: wherever we are now is home.
            state.enc_pos_target = state.enc_pos;
            state.pos_setpoint = state.pos;
            pos_integ = 0.0f;
            carrot_valid = 0; carrot_v = 0.0f; carrot_arrived = 0; /* target re-snapped: carrot restarts at the wheels */
            pos_last_correction = 0.0f;
            vel_damp_filt = 0.0f;
            state.enc_velocity = 0;
            state.enc_vel_reset = 1;

            // Heading: wherever we're pointing now is home.
            yaw_target = (state.phi_right - state.phi_left) / 2.0f;
            state.yaw = yaw_target;
            state.steering_latch = 0.0f;
            state.yaw_latched = 0;

            // Start recovery at the natural balance point,
            // not with an old D2 lean command.
            state.theta_ref = 0.0f;

            // No stale PID history.
            pid_reset(&pitch_pid);
            pid_reset(&yaw_pid);

            // NOW give the motors back.
            if (state.armed)
                motor_hal_standby(0);

            rc_led_set(RC_LED_GREEN, state.armed ? 1 : 0);
            LOG_INFO("IN RANGE — controls rebased, armed=%d", state.armed);
        }

        // else if (was_oob)
        // {
        //     // Back in range: clear the edge, resume standby if still armed, and
        //     // reset both PIDs so whatever the (now-ungated) loop accumulated
        //     // while OOB does not slam through as a kick on the first actuated
        //     // tick -- same clean-restart guarantee the disarm/arm path already
        //     // gives via pid_reset() above.
        //     was_oob = 0;
        //     pid_reset(&pitch_pid);
        //     pid_reset(&yaw_pid);
        //     if (state.armed)
        //         motor_hal_standby(0);
        //     rc_led_set(RC_LED_GREEN, state.armed ? 1 : 0);
        //     LOG_INFO("IN RANGE — armed=%d", state.armed);
        // }

        // ── D2 position (drive) controller ───────────────────────────────
        // When D2 is enabled: the stick command sets a *rate* (ticks/s target).
        // With stick centered: hold enc_pos_target using zone-based lean-angle
        // bias + velocity damping (position-hold-style).
        // When driving: let the bot move, damp with velocity feed-forward.
        // All parameters are in g_pos_config (runtime-tunable via IPC).
        //
        // back_to_spot=1: full zone-based hold (A/B/C/D proportional)
        // back_to_spot=0: only correct inside zone_c — loose hold (position hold mode)
        //
        // max_angle_rate limits how fast the correction can change per tick,
        // preventing D2 from slamming theta_ref and causing oscillation.
        // (reference implementation uses 1°/loop at 500 Hz ≈ 5°/loop at our 100 Hz.)
        // Save raw stick before D2 correction is applied so D2 can tell
        // whether the operator is actually driving.  Must be captured here,
        // before the theta_ref += correction line below mutates it.
        {
            // Track armed transitions for D2 last_correction reset
            static int prev_armed_d2 = 0;
            if (g_controllers.position && state.armed)
            {

                // Reset accumulated correction on re-arm so stale last_correction
                // doesn't slam theta_ref immediately after a fall/recovery.
                if (!prev_armed_d2)
                    pos_last_correction = 0.0f;
                float correction = 0.0f;

                // Use raw stick only — exclude D2's own correction so it doesn't
                // trick the hold logic into thinking the user is driving.
                /* Ask the stick, not the angle.
                 *
                 * This was `fabsf(raw_stick_ref) < 0.5f`, i.e. half a degree of
                 * COMMANDED LEAN. That was ~4% of stick travel back when full
                 * stick meant 11.9 deg. Once sbus drive_scale became
                 * configurable and defaulted to 0.25, full stick meant 2.97 deg
                 * and the same 0.5 deg became 17% of travel -- a dead zone
                 * across the bottom sixth of the stick where the operator is
                 * pushing and position hold is still pulling the other way.
                 *
                 * input_sbus.c has already applied the deadband, so anything
                 * non-zero here is deliberate operator input. The epsilon only
                 * guards float noise. This is now independent of drive_scale,
                 * speed mode and MAX_THETA_REF. */
                /* In target mode there is no "driving" branch to hand over to:
                 * position hold owns theta_ref the whole time, which is the
                 * entire point. Forcing this true keeps the hold loop running
                 * while the stick is deflected, so the bot decelerates into its
                 * moving target instead of being flown open-loop. */
                bool stick_centered = (g_pos_config.drive_mode == DRIVE_MODE_TARGET)
                                          ? true
                                          : (fabsf(raw_stick_norm) < 0.001f);

                // ── D2 verbose debug (enable via IPC: {"type":"debug_position","value":true}) ──
                static uint64_t d2_log_last_us = 0;
                if (g_debug_config.debug_position)
                {
                    uint64_t d2_now_us = rc_nanos_since_boot() / 1000;
                    if (d2_now_us - d2_log_last_us >= 500000) // 2 Hz
                    {
                        d2_log_last_us = d2_now_us;
                        int32_t pos_err = state.enc_pos_target - state.enc_pos;
                        LOG_INFO("[D2] armed=%d D2_en=%d stick_cen=%d raw_stick=%.3f "
                                 "pos=%d tgt=%d err=%d vel=%.2f stopped_vel=%d "
                                 "last_corr=%.4f theta_ref=%.4f",
                                 state.armed,
                                 g_controllers.position,
                                 (int)stick_centered,
                                 raw_stick_ref,
                                 state.enc_pos,
                                 state.enc_pos_target,
                                 pos_err,
                                 state.enc_velocity,
                                 g_pos_config.stopped_vel,
                                 pos_last_correction,
                                 state.theta_ref);
                    }
                }

                if (stick_centered)
                {
                    int32_t err = state.enc_pos_target - state.enc_pos;
                    if (g_pos_config.lead_max > 0)
                    {
                        const float home = (float)state.enc_pos_target;
                        const float here = (float)state.enc_pos;
                        const float lead = (float)g_pos_config.lead_max;
                        if (!carrot_valid)
                        {
                            carrot = here;
                            carrot_v = 0.0f;
                            carrot_arrived = 0;
                            carrot_valid = 1;
                            carrot_home_prev = state.enc_pos_target;
                        }
                        else if (state.enc_pos_target != carrot_home_prev)
                        {
                            /* New target. Re-plan from where the carrot is and
                             * at the speed it already has, rather than snapping
                             * it to the wheels -- a move commanded mid-move
                             * should bend the trajectory, not restart it. */
                            carrot_arrived = 0;
                            carrot_home_prev = state.enc_pos_target;
                        }
                        /* Walk toward home on a trapezoid: speed ramps up at
                         * return_accel, cruises at return_rate, and ramps down
                         * so it arrives at zero speed (v <= sqrt(2 a d)). The
                         * old fixed-step walk started and stopped at full speed. */
                        if (carrot_arrived)
                        {
                            /* Plan finished. The carrot sits on the target and
                             * the loop is an ordinary position hold from here
                             * on -- which is exactly what recovering from an
                             * overshoot wants. Re-planning from a dragged
                             * carrot is what made the overshoot grow. */
                            carrot = home;
                            carrot_v = 0.0f;
                        }
                        else
                        {
                            /* Cruise speed is free. DECELERATION is what
                             * needs lean authority, so the brake clamps the
                             * accel, not the top speed -- clamping the speed
                             * (an earlier try) just made every move crawl.
                             * A lean of t degrees buys g*tan(t) of decel:
                             * 9810 mm/s^2 / 57.3 deg-per-rad / 1.27 mm-per-tick
                             * = 135 enc ticks/s^2 per degree of ACHIEVED lean.
                             * The pitch loop does not achieve what it is asked
                             * for: measured over three runs 2026-09-20 20:44-47
                             * (|setpoint| > 1.5 deg, bot upright, n = 1286-1882
                             * each) the median measured/commanded ratio is
                             * 0.47 / 0.45 / 0.50, at a median duty of only
                             * 0.05-0.08. So a COMMANDED degree is worth about
                             * half that, ~68. Planning against 135 is what let
                             * the trapezoid ask for twice the deceleration the
                             * robot can produce: 2026-09-20 ..937230468, a 158
                             * tick move overshot 335 ticks with posCorr and
                             * vel_damp both pinned for 1.8 s while
                             * pit_measurement sat at ~0 deg against a -2.5 deg
                             * command. The brake never physically happened.
                             * The brake is capped at vel_damp_max degrees and
                             * also has to absorb pushes, so spend 70% of it. The ramp-down
                             * below (v <= sqrt(2 a d)) then starts early
                             * enough that ANY cruise speed stops in the
                             * distance that is left. 2026-09-20 42.0-42.7 it
                             * ran 340 ticks/s with the brake pinned at its
                             * 2 deg cap and went over: that was the plan
                             * asking for more decel than 2 deg can make. */
                            const float vmax = fmaxf(g_pos_config.return_rate, 0.0f);
                            float acc = fmaxf(g_pos_config.return_accel, 1.0f);
                            if (g_pos_config.vel_damp_max > 0.0f)
                            {
                                const float acc_brake = 0.7f * 68.0f *
                                                        g_pos_config.vel_damp_max;
                                acc = fminf(acc, fmaxf(acc_brake, 1.0f));
                            }
                            const float dist = home - carrot;
                            const float dir = (dist > 0.0f) ? 1.0f : (dist < 0.0f ? -1.0f : 0.0f);
                            const float vstop = sqrtf(2.0f * acc * fabsf(dist));
                            const float want = dir * fminf(vmax, vstop);
                            const float dv = acc * DT;
                            if (carrot_v < want) carrot_v = fminf(carrot_v + dv, want);
                            else if (carrot_v > want) carrot_v = fmaxf(carrot_v - dv, want);
                            float next = carrot + carrot_v * DT;
                            if ((dir > 0.0f && next > home) || (dir < 0.0f && next < home) || dir == 0.0f)
                            {
                                next = home;
                                carrot_v = 0.0f;
                                carrot_arrived = 1;   /* and it stays there */
                            }
                            carrot = next;

                            /* Hold the carrot back to within lead_max of the
                             * wheels so it cannot run off and leave the bot
                             * chasing a huge error. Only while the plan is
                             * still running: once it has arrived this clamp is
                             * skipped entirely, so an overshooting bot cannot
                             * drag the target back out and restart the move. */
                            if (!carrot_arrived)
                            {
                                if (carrot > here + lead)
                                    carrot = here + lead;
                                if (carrot < here - lead)
                                    carrot = here - lead;
                            }
                        }
                        /* The spring never sees more than lead_max of error,
                         * however far the bot has strayed. Clamped HERE rather
                         * than by moving the carrot, so a bot that has blown
                         * past the target gets a steady pull back toward it
                         * instead of a rewritten plan. */
                        {
                            float e = carrot - here;
                            if (e > lead)
                                e = lead;
                            if (e < -lead)
                                e = -lead;
                            err = (int32_t)lroundf(e);
                        }
                    }
                    else
                    {
                        carrot_valid = 0;
                        carrot_v = 0.0f;
                        carrot_arrived = 0;
                    }
                    int32_t absErr = abs(err);
                    /* First-order low-pass on the damping term. Filtering here
                     * rather than filtering enc_velocity itself, because
                     * vel_scale_move and the turn turndown read the same signal
                     * and neither has this problem -- only the stop-damping
                     * derivative does. Held across ticks; reset on disarm below
                     * so a stale value cannot kick the first tick after arming. */
                    /* Damp velocity RELATIVE to the carrot's planned speed.
                     * Damping absolute speed meant every move was braked by
                     * v / vel_scale_stop, which the spring (capped at
                     * lead_max / scale) had to overpower: with 40/50 and
                     * vss 4 the bot could never cruise faster than ~6 units
                     * (~60 ticks/s) -- the "way too slow" walk. enc_velocity
                     * is (qpps_m1 + qpps_m2)/10, i.e. enc_pos ticks/s / 10,
                     * so the carrot speed converts with the same /10. At
                     * rest carrot_v is 0 and this is exactly the old term. */
                    /* Feed-forward: the planned speed, but never more than the
                     * bot is actually doing, and only when the two agree in
                     * sign. Taken from enc_velocity directly now that the lead
                     * clamp no longer drags the carrot (the old carrot_v_act
                     * was a proxy for this and stopped meaning anything once
                     * the carrot kept its own plan).
                     *   blocked      -> enc_velocity 0    -> v_ref 0, no phantom lean
                     *   pushed away  -> signs disagree    -> v_ref 0, full damping
                     *   cruising     -> tracks the plan   -> damping ~0
                     *   running away -> capped at the plan -> damping on the excess */
                    float v_ref = 0.0f;
                    if (g_pos_config.lead_max > 0 && carrot_v != 0.0f)
                    {
                        const float plan = carrot_v / 10.0f;   /* enc_velocity units */
                        const float act = (float)state.enc_velocity;
                        if ((plan > 0.0f && act > 0.0f) || (plan < 0.0f && act < 0.0f))
                            v_ref = (plan > 0.0f) ? fminf(plan, act) : fmaxf(plan, act);
                    }
                    float vel_damp = ((float)state.enc_velocity - v_ref) / g_pos_config.vel_scale_stop;

                    {
                        const float fc = g_pos_config.vel_damp_fc;
                        if (fc > 0.0f)
                        {
                            const float dt = 1.0f / (float)SAMPLE_RATE_HZ;
                            const float rc = 1.0f / (2.0f * (float)M_PI * fc);
                            const float a = dt / (dt + rc);
                            vel_damp_filt += a * (vel_damp - vel_damp_filt);
                            vel_damp = vel_damp_filt;
                        }
                        else
                        {
                            vel_damp_filt = vel_damp; /* track, so enabling it mid-run does not step */
                        }
                        /* Cap AFTER filtering: the filter shapes which
                         * frequencies the term acts on, this bounds how far it
                         * can throw theta_ref on any single tick. */
                        const float vdmax = g_pos_config.vel_damp_max;
                        if (vdmax > 0.0f)
                        {
                            if (vel_damp > vdmax)
                                vel_damp = vdmax;
                            if (vel_damp < -vdmax)
                                vel_damp = -vdmax;
                        }
                    }

                    if (absErr < g_pos_config.pos_deadband)
                    {
                        // Inside tight deadband — no position push, but do NOT slam
                        // last_correction to 0. Leaving it intact lets the rate limiter
                        // below ease the correction out at max_angle_rate instead of
                        // dropping it discontinuously, and stops the ramp having to
                        // rebuild from zero on the way back out. Damping is unaffected
                        // (it is applied after the limiter and is no longer gated).
                        correction = 0.0f;
                        state.pos_scale = 0.0f;
                    }
                    else if (g_pos_config.back_to_spot)
                    {
                        // Full zone-based proportional hold
                        if (absErr > g_pos_config.zone_a)
                            state.pos_scale = g_pos_config.scale_a;
                        else if (absErr > g_pos_config.zone_b)
                            state.pos_scale = g_pos_config.scale_b;
                        else if (absErr > g_pos_config.zone_c)
                            state.pos_scale = g_pos_config.scale_c;
                        else
                            state.pos_scale = g_pos_config.scale_d;

                        correction = (float)err / state.pos_scale;
                    }
                    else
                    {
                        if (absErr < g_pos_config.zone_c)
                        {
                            state.pos_scale = g_pos_config.scale_d;
                            correction = (float)err / state.pos_scale;
                        }
                        else
                        {
                            // Outside zone_c in loose-hold mode: abandon the old
                            // target rather than correcting toward it.
                            state.pos_scale = 0.0f;
                            state.enc_pos_target = state.enc_pos;
                            pos_integ = 0.0f; /* target moved — accumulated error is stale */
                            carrot_valid = 0; carrot_v = 0.0f; carrot_arrived = 0; /* target re-snapped: carrot restarts at the wheels */
                        }
                    }

                    state.pos_correction = correction;

                    /* Integral. Accumulated in degrees so the clamp is in the
                     * same units as max_correction. Applied BEFORE the rate
                     * limiter on purpose: it is a slow term and belongs on the
                     * smooth path, unlike vel_damp which is deliberately
                     * immediate. Conditional integration -- frozen while the
                     * output sits at a clamp, so it cannot wind up against a
                     * limit and lurch when the limit releases.
                     *
                     * The test is on the OUTPUT clamp only. An earlier
                     * version also froze on "the carrot is at full lead", which
                     * was wrong and made the robot park short: the spring being
                     * at ITS cap is precisely when the integral is supposed to
                     * be adding the authority the proportional term has run out
                     * of. Measured 2026-09-20 bbot_ring_1789950092231: stopped
                     * 150 ticks (190 mm) from the target with posCorr pinned,
                     * pos_integ frozen at +0.325 for 3.2 s while thetaAdj was
                     * 3.21 against a max_correction of 6 -- 2.8 deg of unused
                     * headroom and the one term that could break stiction held
                     * shut. 16% of that run was spent parked >60 ticks off
                     * target with the wheels stationary. */
                    if (g_pos_config.pos_ki > 0.0f && state.pos_scale > 0.0f)
                    {
                        const float dt = 1.0f / (float)SAMPLE_RATE_HZ;
                        const int clamped = fabsf(pos_last_correction) >=
                                            g_pos_config.max_correction * 0.98f;
                        /* Freeze only against the limit; unwinding is allowed,
                         * so the term still recovers the moment the error
                         * changes sign. */
                        const int pushing_out = (pos_integ >= 0.0f) == (err >= 0);
                        if (!(clamped && pushing_out))
                            pos_integ += g_pos_config.pos_ki * (float)err * dt;
                        const float ilim = g_pos_config.pos_i_max;
                        if (ilim > 0.0f)
                        {
                            if (pos_integ > ilim)
                                pos_integ = ilim;
                            if (pos_integ < -ilim)
                                pos_integ = -ilim;
                        }
                        correction += pos_integ;
                    }
                    else
                    {
                        pos_integ = 0.0f;
                    }
                    state.pos_i_term = -pos_integ; /* sign matches thetaAdj */

                    // Rate-limit the position correction — prevents slamming theta_ref
                    float delta = correction - pos_last_correction;
                    if (delta > g_pos_config.max_angle_rate)
                        delta = g_pos_config.max_angle_rate;
                    if (delta < -g_pos_config.max_angle_rate)
                        delta = -g_pos_config.max_angle_rate;
                    correction = pos_last_correction + delta;
                    pos_last_correction = correction;

                    // Apply vel_damp after rate limiter so decel/accel acts at full speed.
                    //
                    // UNGATED. This was previously `if (absErr > 5)`, which cut velocity
                    // damping inside +/-5 ticks — 23% of a typical run, centred exactly on
                    // the target. The bot arrived at the target with no brakes, coasted
                    // through at full speed (5.00 ticks/100ms inside the zone vs 5.18
                    // outside) and rang at a fixed ~2.9s period that no value of scale_d
                    // could touch, because scale_d does not reach into the dead zone.
                    // The suppressed damping averaged 1.000 deg and peaked at 2.200 deg
                    // against a mean position correction of 0.286 deg.
                    //
                    // Note the interaction with the absErr < 2 deadband above: there
                    // correction and last_correction are both forced to 0, so inside the
                    // deadband the output is now pure damping — brake at the target, no
                    // position push. That is the intent. Do not re-add a gate here.
                    correction -= vel_damp;
                    state.pos_vel_damp = -vel_damp;

                    // Hard clamp
                    if (correction > g_pos_config.max_correction)
                        correction = g_pos_config.max_correction;
                    if (correction < -g_pos_config.max_correction)
                        correction = -g_pos_config.max_correction;

                    /* Negated on purpose. enc_pol_l/enc_pol_r are -1.0 in
                     * robot.conf, so enc_pos counts negative when the bot moves
                     * forward and (enc_pos_target - enc_pos) comes out with the
                     * wrong sign for a lean command.
                     *
                     * Do NOT "fix" this by flipping enc_pol to +1 instead. The
                     * yaw loop closes on (phi_right - phi_left) -- see the
                     * yaw_target line further up and pid_update(&yaw_pid, ...)
                     * in the mixer. Flipping both encoder polarities flips that
                     * difference too, turns steering into positive feedback,
                     * and the bot spins the moment it arms. Yaw owns enc_pol;
                     * position compensates here. */
                    state.pos_output = -correction;
                    state.theta_ref = -correction;
                }
                else
                {
                    // Driving — back-EMF compensation to smooth acceleration
                    float vel_comp = (float)state.enc_velocity / g_pos_config.vel_scale_move;
                    if ((state.theta_ref > 0.0f && state.enc_velocity < 0) ||
                        (state.theta_ref < 0.0f && state.enc_velocity > 0) ||
                        state.theta_ref == 0.0f)
                    {
                        correction += vel_comp;
                    }
                    state.enc_pos_target = state.enc_pos;
                    pos_integ = 0.0f; /* target moved — accumulated error is stale */
                    carrot_valid = 0; carrot_v = 0.0f; carrot_arrived = 0; /* target re-snapped: carrot restarts at the wheels */

                    // Position hold is not running while driving. Clear the hold
                    // telemetry so it does not sit at stale values from the last
                    // centred tick and get read as live position-hold activity.
                    state.pos_correction = 0.0f;
                    state.pos_vel_damp = 0.0f;
                    state.pos_i_term = 0.0f;
                    vel_damp_filt = 0.0f;
                    pos_integ = 0.0f;
                    state.pos_scale = 0.0f;
                    state.pos_output = correction;
                }
            } // end if (g_controllers.position && state.armed)
            else
            {
                // D2 disabled — keep target synced so it's ready when re-enabled
                if (g_debug_config.debug_position)
                {
                    static uint64_t d2_else_last_us = 0;
                    uint64_t d2_now_us = rc_nanos_since_boot() / 1000;
                    if (d2_now_us - d2_else_last_us >= 500000)
                    {
                        d2_else_last_us = d2_now_us;
                        LOG_INFO("[D2-ELSE] D2_drive=%d armed=%d enc_pos_target synced to %d",
                                 g_controllers.position,
                                 state.armed,
                                 state.enc_pos);
                    }
                }
                state.enc_pos_target = state.enc_pos;
                pos_integ = 0.0f; /* target moved — accumulated error is stale */
                carrot_valid = 0; carrot_v = 0.0f; carrot_arrived = 0; /* target re-snapped: carrot restarts at the wheels */
                state.pos_setpoint = state.pos;

                // Turning D2 off zeroed g_controllers.position, but nothing here
                // ever touched state.theta_ref: it stayed pinned at whatever lean
                // the hold loop last commanded (state.theta_ref = -correction,
                // above). "Position off" then meant "position's last correction,
                // frozen forever" -- the balance loop kept chasing a stale
                // nonzero setpoint, which reads as the bot pushing one way or
                // another for no reason while trying to isolate D1 for tuning.
                // Ease it back to zero at the same max_angle_rate every other
                // theta_ref change in this file already respects, rather than
                // slamming it -- a balancing robot does not want a discontinuous
                // step in its angle reference.
                {
                    float d = 0.0f - state.theta_ref;
                    float lim = g_pos_config.max_angle_rate;
                    if (d > lim)
                        d = lim;
                    if (d < -lim)
                        d = -lim;
                    state.theta_ref += d;
                }
            }
            prev_armed_d2 = state.armed;
        } // end D2 block

        // ── D2/D3 diagnostic log (2 Hz, always-on while armed) ───────────
        if (state.armed)
        {
            static uint64_t diag_last_us = 0;
            uint64_t diag_now_us = rc_nanos_since_boot() / 1000;
            if (diag_now_us - diag_last_us >= 500000)
            {
                diag_last_us = diag_now_us;
                int32_t pos_err = state.enc_pos_target - state.enc_pos;
                float phi_diff = (state.phi_left - state.phi_right) / 2.0f;
                LOG_INFO("[POS] en=%d pos=%d tgt=%d err=%d vel=%.2f corr_out=%.3f theta_ref=%.3f",
                         g_controllers.position,
                         state.enc_pos, state.enc_pos_target, pos_err,
                         state.enc_velocity,
                         state.pos_output,
                         state.theta_ref);
                LOG_INFO("[steering] en=%d psi=%.2f phi_diff=%.2f steering=%.3f",
                         g_controllers.yaw,
                         state.psi, phi_diff, state.yaw);
            }
        }

        // ── Pose: a commanded lean, ramped ───────────────────────────────
        // Overrides whatever the hold loop or stick asked for. Ramped at
        // max_angle_rate in BOTH directions so dialling it in eases the bot into
        // the lean and zeroing it eases back out — stepping a balancing robot's
        // angle reference is how you put it on its face.
        //
        // DISTINCT from theta_offset, which is the balance trim (where upright
        // IS). This deliberately leans the bot AWAY from upright so you can watch
        // it creep at a known angle; it WILL drive away.
        {
            static float pose_ramp = 0.0f;

            /* Never resume a stale pose on re-arm. */
            if (!state.armed)
            {
                pose_ramp = 0.0f;
                state.pose_lean = 0.0f;
            }

            float d = state.pose_lean - pose_ramp;
            float lim = g_pos_config.max_angle_rate;
            if (d > lim)
                d = lim;
            if (d < -lim)
                d = -lim;
            pose_ramp += d;

            /* Only take over once there is something to apply, so a zero pose
             * leaves the hold loop completely untouched. */
            if (fabsf(state.pose_lean) > 0.0005f || fabsf(pose_ramp) > 0.0005f)
                state.theta_ref = pose_ramp;
        }

        // Saturate references before the next ISR reads them
        rc_saturate_float(&state.theta_ref, -MAX_THETA_REF, MAX_THETA_REF);
        /* state.yaw is now a HEADING TARGET in degrees of phi_diff, not a
         * normalised -1..1 command, so the old +/-MAX_YAW_CMD clamp would have
         * pinned it at 1 degree and undone the rate integration entirely.
         *
         * Clamp it near the CURRENT heading instead. That still allows unlimited
         * continuous turning (the target moves with the bot) while stopping the
         * target running away if the wheels are blocked or the bot is picked up --
         * which would otherwise spin it up the moment it regained traction. */
        {
            /* Anti-windup only. This used to write the clamped value back
             * into state.yaw, which meant a robot twisted more than
             * MAX_YAW_LEAD had its target DRAGGED along behind it and the
             * original heading was gone for good -- the same defect as the
             * position carrot's lead clamp. The bound on how hard the loop
             * may pull is what matters, so bound the lead here and leave the
             * target alone; robot_yaw_error() applies it. Log
             * bbot_ring_1789950092231 shows |yaw_error| pinned at exactly
             * 90.000, the clamp, rather than a heading being recovered. */
            float phi_now = (state.phi_right - state.phi_left) / 2.0f;
            float lead = state.yaw - phi_now;
            if (lead > MAX_YAW_LEAD || lead < -MAX_YAW_LEAD)
                LOG_DEBUG("steering: heading lead %.1f deg beyond +/-%.0f clamp",
                          lead, MAX_YAW_LEAD);
        }

        // ── Telemetry & display ───────────────────────────────────────────
        telemetry_update();

        g_telemetry_data.motors.left_duty = last_left_duty;
        g_telemetry_data.motors.right_duty = last_right_duty;

        /* Full loop rate, straight into RAM. Must come after the motor duties
         * are stamped above or every logged row would carry the previous
         * tick's command. */
        ring_tick();

        uint64_t now_sched_us = rc_nanos_since_boot() / 1000; // microseconds

        /* Scheduling in MICROSECONDS with a phase accumulator, not milliseconds
         * with a "have we waited long enough" test.
         *
         * The old form was
         *     interval = 1000 / hz;                 // integer ms
         *     if (now_ms - last_ms >= interval) { ...; last_ms = now_ms; }
         * and it silently delivered the wrong rate. Two compounding errors:
         *
         *   1. 1000/30 truncates to 33 ms.
         *   2. last_ms is reset to the tick that noticed, not to when the packet
         *      was due, so the period is rounded UP to a whole loop tick. With a
         *      10 ms tick a 33 ms interval fires every 40 ms.
         *
         * Asking for 30 Hz therefore produced 25 Hz -- measured on the bot as
         * 24.4 Hz, alongside a correct 10 Hz telemetry stream, which is what put
         * us onto this. Only rates that divide the loop period came out right,
         * so 30/40/60 were all quietly wrong while 10/20/50 were fine.
         *
         * Advancing a due-time by the exact period keeps the AVERAGE rate right
         * even when the period is not a multiple of the tick: 30 Hz becomes an
         * alternating 30/40 ms, which averages 33.3 ms. Rates that do divide the
         * tick (25, 50) stay perfectly even, which is why they make better
         * defaults for anything being graphed. */
        {
            int tel_hz = g_debug_config.rates.pid_states;
            if (tel_hz < 1)
                tel_hz = 1;
            if (tel_hz > SAMPLE_RATE_HZ)
                tel_hz = SAMPLE_RATE_HZ;
            uint64_t period_us = 1000000ULL / (uint64_t)tel_hz;

            if (now_sched_us >= next_telemetry_us)
            {
                ipc_broadcast_telemetry();
                telemetry_counter++;
                /* Resync rather than burst-catch-up if we fell far behind (a
                 * long stall, or first pass when next_ is 0). Firing three
                 * packets back to back to "make up" lost time would just push a
                 * spike into a client that is already struggling. */
                next_telemetry_us = (now_sched_us - next_telemetry_us > period_us)
                                        ? now_sched_us + period_us
                                        : next_telemetry_us + period_us;
            }
        }

        /* RC on its own, faster clock. Separate from telemetry because the SBUS
         * frame rate (~143 Hz) is an order of magnitude above the telemetry
         * rate: bundled into the 10 Hz packet, 13 of every 14 receiver frames
         * were thrown away before they ever left the board. The packet is small
         * (~280 bytes) so this is cheaper than it sounds. */
        {
            int rc_hz = g_debug_config.rates.rc;
            if (rc_hz < 1)
                rc_hz = 1;
            if (rc_hz > SAMPLE_RATE_HZ)
                rc_hz = SAMPLE_RATE_HZ; /* above the loop
                                         * rate we would resend identical data */
            uint64_t period_us = 1000000ULL / (uint64_t)rc_hz;

            if (now_sched_us >= next_rc_us)
            {
                ipc_broadcast_rc();
                next_rc_us = (now_sched_us - next_rc_us > period_us)
                                 ? now_sched_us + period_us
                                 : next_rc_us + period_us;
            }
        }

        /* Config: sent on client connect and whenever a command changed one of
         * motor_config / pos_config / sbus_config. No-op on the vast majority of
         * iterations -- it is a flag test. */
        ipc_broadcast_config_if_dirty();

        display_update();

        /* Outside the tick: writes the file once, when the capture fills. */
        /* Report the sensor watchdog from here, not the ISR. */
        {
            static int stale_logged = 0;
            if (g_imu_stale && !stale_logged)
            {
                stale_logged = 1;
                LOG_WARN("IMU STALE -- theta stuck at %.2f. Motors cut, will not "
                         "actuate until it moves again.",
                         state.theta);
            }
            else if (!g_imu_stale && stale_logged)
            {
                stale_logged = 0;
                LOG_WARN("IMU recovered -- theta moving again.");
            }
        }


        loop_counter++;

        uint64_t elapsed_us = rc_nanos_since_boot() / 1000 - now_us;
        if (elapsed_us < 10000)
            rc_usleep(10000 - elapsed_us);
    }
}

/**
 * @brief Cleanup robot hardware
 */
void robot_cleanup(void)
{
    LOG_INFO("Cleaning up robot...");

    motor_hal_coast(); /* off on the way out, not "hold zero speed" */
    motor_hal_cleanup();
    rc_mpu_power_off();

    rc_led_set(RC_LED_GREEN, 0);
    rc_led_set(RC_LED_RED, 0);

    rc_remove_pid_file();
    rc_set_state(EXITING);

    LOG_INFO("Robot cleanup complete");
}