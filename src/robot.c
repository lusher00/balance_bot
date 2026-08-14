// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
/**
 * @file robot.c
 * @brief Main robot control loop with telemetry and IPC integration
 */

#include "balance_bot.h"
#include "motor_hal.h"
#include "display.h"
#include <stdio.h>
#include <stdlib.h>
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

// Debug configuration (defined in debug_config.h, initialized in main.c)
debug_config_t g_debug_config;

// Telemetry counters
static uint64_t telemetry_counter = 0;
static uint64_t last_telemetry_broadcast = 0;

// Motor duty tracking (for telemetry)
static float last_left_duty = 0.0f;
static float last_right_duty = 0.0f;

// Motor output handoff: ISR writes, main loop applies to hardware.
// volatile prevents compiler optimizing away cross-context reads/writes.
static volatile float pending_left_duty = 0.0f;
static volatile float pending_right_duty = 0.0f;
static volatile int motor_output_ready = 0;

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

    // Motors only run when armed AND in safe angle range (trying)
    if (!state.armed || !state.trying)
    {
        pending_left_duty = 0.0f;
        pending_right_duty = 0.0f;
        motor_output_ready = 0;
        return;
    }

    // PID — setpoints and encoder positions are maintained by the main loop.
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
        yaw_output = pid_update(&yaw_pid, state.yaw, phi_diff);
    }
    else
    {
        yaw_output = state.yaw;
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

    g_motor_config.mode = MOTOR_HAL_MODE_DEFAULT;
    g_motor_config.qpps_max = MOTOR_QPPS_MAX_DEFAULT;
    g_motor_config.accel_qpps = MOTOR_ACCEL_QPPS_DEFAULT;
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
void robot_run(void)
{
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

        // ── Disarm handling ───────────────────────────────────────────────
        static int prev_armed = 0;
        if (prev_armed && !state.armed)
        {
            // Disarm: stop motors, reset PIDs. No estop — that's only for E-STOP button.
            pending_left_duty = 0.0f;
            pending_right_duty = 0.0f;
            motor_output_ready = 0;
            motor_hal_set_both(0.0f, 0.0f);
            pid_reset(&pitch_pid);
            pid_reset(&yaw_pid);
            last_left_duty = 0.0f;
            last_right_duty = 0.0f;
            state.enc_pos_target = state.enc_pos;
        }
        if (!prev_armed && state.armed)
        {
            state.enc_pos_target = state.enc_pos;
            state.theta_ref = 0.0f;
            // If already in bounds when armed, enable motor output immediately
            if (fabsf(state.theta - state.theta_offset) < 14.0f)
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
            /* Hardware speed from RoboClaw (GETM1SPEED/GETM2SPEED, cmds 18/19).
             * Throttled to POS_VEL_PERIOD_MS so downstream D2 math still
             * operates on the same ~100 ms window it was tuned against.
             * Divide sum by 10 to convert QPPS (pulses/s) → ticks/100ms so
             * vel_scale_stop values remain in the same range as before.       */
            static uint64_t last_vel_us = 0;
            uint64_t now_us = rc_nanos_since_boot() / 1000;

            if (state.enc_vel_reset)
            {
                state.enc_velocity = 0;
                state.enc_velocity_raw = 0;
                last_vel_us = now_us;
                state.enc_vel_reset = 0;
            }
            else if ((now_us - last_vel_us) >= (POS_VEL_PERIOD_MS * 1000ULL))
            {
                int32_t m1 = 0, m2 = 0;
                if (motor_hal_read_encoder_speeds(&m1, &m2) == 0)
                {
                    /* Sum both motors (same convention as enc_pos = L + R),
                     * scale to ticks/100ms for backward-compat with vel_scale */
                    state.enc_velocity_raw = (float)(m1 + m2);
                    /* 10.0f not 10: m1/m2 are int32_t, so integer division here
                     * would quantize to whole ticks/100ms BEFORE the assignment and
                     * the float field would gain nothing. At vel_scale_stop=5 each
                     * lost tick was 0.2 deg of theta_ref. */
                    state.enc_velocity = (float)(m1 + m2) / 10.0f;
                }
                last_vel_us = now_us;
            }
        }

        // ── Input sources → theta_ref / steering ──────────────────────────
        if (state.mode == MODE_EXT_INPUT)
        {
            input_packet_t pkt;
            if (uart_input_get(&pkt))
            {
                state.ext_input = pkt;
                state.theta_ref = -pkt.y * MAX_THETA_REF;
                state.yaw = pkt.x * MAX_YAW_CMD;
            }
            else
            {
                state.theta_ref = 0.0f;
                state.yaw = 0.0f;
            }
        }

        xbox_update();
        sbus_update();

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
        static float yaw_target = 0.0f;

        /* Fractional carry for target-mode drive: at 100 Hz a full-stick
         * advance is only a few ticks per loop, so truncating every tick would
         * throw most of the command away. */
        static float drive_accum = 0.0f;

        if (sbus_is_connected() && state.armed && state.mode == MODE_BALANCE)
        {
            stick_norm = sbus_get_drive();

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
            yaw_target += sbus_get_turn() * g_sbus_config.turn_rate * DT;
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
        else if (!sbus_is_connected())
        {
            stick_input = 0.0f;
            stick_norm = 0.0f;
        }

        /* Disarmed or unlinked: forget the accumulated heading so re-arming does
         * not immediately spin toward a target set minutes ago. */
        if (!state.armed || !sbus_is_connected())
        {
            yaw_target = (state.phi_right - state.phi_left) / 2.0f;
            state.yaw = yaw_target;
        }

        // Capture stick input before D2 adds its correction.
        // theta_ref may have accumulated D2 correction from previous loops,
        // so raw_stick_ref must reflect only the operator's input.
        float raw_stick_ref = stick_input;
        float raw_stick_norm = stick_norm;

        // ── D3 steering latch ─────────────────────────────────────────────
        // When the turn stick returns to centre AFTER an active turn, latch
        // the current phi_diff as the D3 setpoint so the bot holds its heading
        // instead of fighting back to zero.
        //
        // The latch only fires on a non-zero→zero transition.  With SBUS
        // disconnected (steering always 0), was_turning is never set and the
        // latch never fires, so D3 always targets phi_diff=0 (go straight).
        {
            float phi_diff = (state.phi_left - state.phi_right) / 2.0f;
            bool turn_centered = (fabsf(state.yaw) < 0.05f);
            static bool was_turning = false;

            if (!turn_centered)
            {
                // Active turn input — mark that we were turning and clear latch
                was_turning = true;
                state.yaw_latched = 0;
                state.steering_latch = 0.0f;

                // Velocity-based turning authority reduction
                if (g_pos_config.vel_scale_turning > 0.0f)
                {
                    float vel_turndown = fabsf((float)state.enc_velocity /
                                               g_pos_config.vel_scale_turning);
                    if (state.yaw < 0.0f)
                    {
                        state.yaw += vel_turndown;
                        if (state.yaw > 0.0f)
                            state.yaw = 0.0f;
                    }
                    else if (state.yaw > 0.0f)
                    {
                        state.yaw -= vel_turndown;
                        if (state.yaw < 0.0f)
                            state.yaw = 0.0f;
                    }
                }
            }
            else if (was_turning && !state.yaw_latched)
            {
                // Stick just returned to centre after an active turn — latch now
                state.steering_latch = phi_diff;
                state.yaw_latched = 1;
                was_turning = false;
            }

            if (state.yaw_latched && g_controllers.yaw)
                state.yaw = state.steering_latch;
        }

        // ── Fall detection / auto-recovery ────────────────────────────────
        // trying  = angle is within recoverable range (<10°), managed automatically
        // armed   = operator intent, set/cleared by IPC only — never auto-set here
        //
        // States:
        //   armed=1, trying=1  → running normally
        //   armed=0, trying=1  → operator disarmed but angle ok (standby)
        //   armed=0, trying=0  → out of range / fell — motors off
        //   armed=1, trying=0  → can't happen (arm requires trying=1)
        float eff_angle = fabsf(state.theta - state.theta_offset);
        if (eff_angle > 15.0f)
        {
            // OOB: always stop motors regardless of whether trying was already 0.
            // (e.g. angle passed through the 10–15° zone first, clearing trying
            // without stopping motors — then the old if(trying) guard was a no-op.)
            motor_hal_set_both(0.0f, 0.0f);
            motor_hal_standby(1);
            if (state.trying)
            {
                state.trying = 0;
                rc_led_set(RC_LED_GREEN, 0);
                state.enc_pos_target = state.enc_pos;
                state.enc_velocity = 0;
                state.enc_vel_reset = 1;
                state.theta_ref = 0.0f;
                if (state.armed)
                    LOG_WARN("OOB — motors cut (eff=%.1f deg), staying armed", eff_angle);
            }
        }
        else if (eff_angle < 10.0f)
        {
            if (!state.trying)
            {
                state.trying = 1;
                if (state.armed)
                    motor_hal_standby(0);
                LOG_INFO("IN RANGE — trying=1, armed=%d", state.armed);
            }
        }
        else
        {
            // Between 10° and 15° — clear trying so arm is blocked until stable.
            // Must also explicitly stop motors: ISR will stop queuing commands
            // (motor_output_ready stays 0) but the H-bridge holds its last value
            // unless we zero it here.
            if (state.trying)
            {
                state.trying = 0;
                motor_hal_set_both(0.0f, 0.0f);
            }
        }

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
                static float last_correction = 0.0f;

                // Reset accumulated correction on re-arm so stale last_correction
                // doesn't slam theta_ref immediately after a fall/recovery.
                if (!prev_armed_d2)
                    last_correction = 0.0f;
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
                                 last_correction,
                                 state.theta_ref);
                    }
                }

                if (stick_centered)
                {
                    int32_t err = state.enc_pos_target - state.enc_pos;
                    int32_t absErr = abs(err);
                    float vel_damp = (float)state.enc_velocity / g_pos_config.vel_scale_stop;

                    if (absErr < 2)
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
                        }
                    }

                    state.pos_correction = correction;

                    // Rate-limit the position correction — prevents slamming theta_ref
                    float delta = correction - last_correction;
                    if (delta > g_pos_config.max_angle_rate)
                        delta = g_pos_config.max_angle_rate;
                    if (delta < -g_pos_config.max_angle_rate)
                        delta = -g_pos_config.max_angle_rate;
                    correction = last_correction + delta;
                    last_correction = correction;

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

                    state.pos_output = correction;
                    state.theta_ref = correction;
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

                    // Position hold is not running while driving. Clear the hold
                    // telemetry so it does not sit at stale values from the last
                    // centred tick and get read as live position-hold activity.
                    state.pos_correction = 0.0f;
                    state.pos_vel_damp = 0.0f;
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
                state.pos_setpoint = state.pos;
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
            if (d > lim) d = lim;
            if (d < -lim) d = -lim;
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
            float phi_now = (state.phi_right - state.phi_left) / 2.0f;
            float lead = state.yaw - phi_now;
            if (lead > MAX_YAW_LEAD)
                state.yaw = phi_now + MAX_YAW_LEAD;
            else if (lead < -MAX_YAW_LEAD)
                state.yaw = phi_now - MAX_YAW_LEAD;
        }

        // ── Telemetry & display ───────────────────────────────────────────
        telemetry_update();

        g_telemetry_data.motors.left_duty = last_left_duty;
        g_telemetry_data.motors.right_duty = last_right_duty;

        uint64_t now = rc_nanos_since_boot() / 1000000; // ms
        uint64_t telemetry_interval = 1000 / g_debug_config.rates.pid_states;

        if (now - last_telemetry_broadcast >= telemetry_interval)
        {
            ipc_broadcast_telemetry();
            last_telemetry_broadcast = now;
            telemetry_counter++;
        }

        display_update();

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

    motor_hal_set_both(0.0f, 0.0f);
    motor_hal_cleanup();
    rc_mpu_power_off();

    rc_led_set(RC_LED_GREEN, 0);
    rc_led_set(RC_LED_RED, 0);

    rc_remove_pid_file();
    rc_set_state(EXITING);

    LOG_INFO("Robot cleanup complete");
}