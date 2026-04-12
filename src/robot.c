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
#include <roboclaw_estop.h>

/* Process state — used by rc_get_state()/rc_set_state() in rc_compat.h */
volatile rc_state_t g_rc_state = UNINITIALIZED;

// Global state
robot_state_t state = {0};
rc_mpu_data_t mpu_data;
pid_controller_t balance_pid, drive_pid, steering_pid;

// Runtime-tunable position controller parameters (initialised in robot_init)
pos_config_t g_pos_config;

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

    if (!state.armed)
        return;

    // PID — setpoints and encoder positions are maintained by the main loop.
    float balance_output = 0.0f;
    float steering_output = 0.0f;

    if (g_debug_config.controllers.D1_balance)
    {
        balance_output = pid_update(&balance_pid,
                                    state.theta_ref + state.theta_offset,
                                    state.theta);
    }

    if (g_debug_config.controllers.D3_steering)
    {
        float phi_diff = (state.phi_left - state.phi_right) / 2.0f;
        steering_output = pid_update(&steering_pid, state.steering, phi_diff);
    }
    else
    {
        steering_output = state.steering;
    }

    float left_duty = balance_output - steering_output;
    float right_duty = balance_output + steering_output;
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
    pid_init(&balance_pid, BALANCE_KP, BALANCE_KI, BALANCE_KD, DT);
    pid_init(&drive_pid, DRIVE_KP, DRIVE_KI, DRIVE_KD, DT);
    pid_init(&steering_pid, STEERING_KP, STEERING_KI, STEERING_KD, DT);

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

    g_motor_config.mode = MOTOR_HAL_MODE_DEFAULT;
    g_motor_config.qpps_max = MOTOR_QPPS_MAX_DEFAULT;
    g_motor_config.accel_qpps = MOTOR_ACCEL_QPPS_DEFAULT;
    g_motor_config.pol_l = 1.0f;
    g_motor_config.pol_r = 1.0f;
    LOG_INFO("PID Controllers:");
    LOG_INFO("  D1_balance:  Kp=%.3f Ki=%.3f Kd=%.3f", BALANCE_KP, BALANCE_KI, BALANCE_KD);
    LOG_INFO("  D2_balance:  Kp=%.3f Ki=%.3f Kd=%.3f", DRIVE_KP, DRIVE_KI, DRIVE_KD);
    LOG_INFO("  D3_steering: Kp=%.3f Ki=%.3f Kd=%.3f", STEERING_KP, STEERING_KI, STEERING_KD);

    // IMU config
    imu_offsets_load(&g_imu_offsets);

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
    state.steering = 0.0f;

    rc_make_pid_file();
    rc_set_state(RUNNING);

    LOG_INFO("Robot initialized successfully");
    return 0;
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

        // ── Disarm handling ───────────────────────────────────────────────
        static int prev_armed = 0;
        if (prev_armed && !state.armed)
        {
            motor_output_ready = 0;
            motor_hal_set_both(0.0f, 0.0f);
            pid_reset(&balance_pid);
            pid_reset(&drive_pid);
            pid_reset(&steering_pid);
            last_left_duty = 0.0f;
            last_right_duty = 0.0f;
            state.enc_pos_target = state.enc_pos; /* D2: snap target on disarm */
        }
        if (!prev_armed && state.armed)
        {
            state.enc_pos_target = state.enc_pos; /* D2: snap target on arm */
            state.theta_ref = 0.0f;               /* D2: start correction from zero */
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

        // ── Encoder position + velocity (100 ms window) ───────────────────
        state.enc_pos = left_ticks + right_ticks;
        {
            static int32_t last_enc_pos = 0;
            static uint64_t last_vel_us = 0;
            static bool vel_init = false;

            uint64_t now_us = rc_nanos_since_boot() / 1000;

            if (state.enc_vel_reset)
            {
                last_enc_pos = state.enc_pos; // == 0 after zero_encoders
                last_vel_us = now_us;
                vel_init = true;
                state.enc_vel_reset = 0;
            }
            else if (!vel_init)
            {
                last_enc_pos = state.enc_pos;
                last_vel_us = now_us;
                vel_init = true;
            }
            else if ((now_us - last_vel_us) >= (POS_VEL_PERIOD_MS * 1000ULL))
            {
                int32_t delta = state.enc_pos - last_enc_pos;
                state.enc_velocity_raw = delta;
                state.enc_velocity = delta;
                last_enc_pos = state.enc_pos;
                last_vel_us = now_us;

                // enc_pos_target is set on arm and updated by the driving branch.
                // No latch here — the velocity block must not chase the bot.
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
                state.steering = pkt.x * MAX_STEERING;
            }
            else
            {
                state.theta_ref = 0.0f;
                state.steering = 0.0f;
            }
        }

        xbox_update();
        sbus_update();

        // ── SBUS drive commands (no arming via SBUS — IPC only) ───────────
        // stick_input tracks operator drive intent only — never touched by D2.
        // This is what raw_stick_ref must read, not state.theta_ref which
        // accumulates D2 correction across loops.
        static float stick_input = 0.0f;
        if (sbus_is_connected() && state.armed && state.mode == MODE_BALANCE)
        {
            stick_input = sbus_get_drive() * MAX_THETA_REF;
            state.theta_ref = stick_input;
            state.steering = sbus_get_turn() * MAX_STEERING;
        }
        else if (!sbus_is_connected())
        {
            stick_input = 0.0f;
        }

        // Capture stick input before D2 adds its correction.
        // theta_ref may have accumulated D2 correction from previous loops,
        // so raw_stick_ref must reflect only the operator's input.
        float raw_stick_ref = stick_input;

        // ── D3 steering latch ─────────────────────────────────────────────
        // When the turn stick returns to centre, latch the current phi_diff
        // as the D3 setpoint.  This prevents D3 from fighting back to zero
        // after the bot comes to rest at an asymmetric wheel position.
        //
        // Also applies position-hold-style turning velocity scale: reduces turning
        // authority at speed so the bot doesn't spin out. The raw stick value
        // is reduced by |enc_velocity| / vel_scale_turning, clamped back toward
        // zero (never past it).
        {
            float phi_diff = (state.phi_left - state.phi_right) / 2.0f;
            bool turn_centered = (fabsf(state.steering) < 0.05f);

            if (turn_centered && !state.steering_latched)
            {
                state.steering_latch = phi_diff;
                state.steering_latched = 1;
            }
            else if (!turn_centered)
            {
                state.steering_latched = 0;
                state.steering_latch = 0.0f;

                // Velocity-based turning authority reduction (position hold pattern):
                // scale down turning command at high wheel speed so the bot doesn't
                // spin out. The reduction is subtracted in the direction of the
                // command, clamped so it never reverses sign.
                if (g_pos_config.vel_scale_turning > 0.0f)
                {
                    float vel_turndown = fabsf((float)state.enc_velocity /
                                               g_pos_config.vel_scale_turning);
                    if (state.steering < 0.0f)
                    {
                        state.steering += vel_turndown;
                        if (state.steering > 0.0f)
                            state.steering = 0.0f;
                    }
                    else if (state.steering > 0.0f)
                    {
                        state.steering -= vel_turndown;
                        if (state.steering < 0.0f)
                            state.steering = 0.0f;
                    }
                }
            }

            if (state.steering_latched && g_debug_config.controllers.D3_steering)
                state.steering = state.steering_latch;
        }

        // ── Fall detection / auto-recovery ────────────────────────────────
        float eff_angle = fabsf(state.theta - state.theta_offset);
        if (state.armed && eff_angle > 15.0f)
        {
            // Fell — cut motors. Stay trying so recovery re-arms automatically.
            state.armed = 0;
            motor_output_ready = 0;
            motor_hal_set_both(0.0f, 0.0f);
            motor_hal_standby(1);
            rc_led_set(RC_LED_GREEN, 0);
            // Reset encoders and theta_ref so re-arm starts from a clean position.
            motor_hal_encoder_reset_all();
            state.enc_left = 0;
            state.enc_right = 0;
            state.enc_pos = 0;
            state.enc_pos_target = 0;
            state.enc_velocity = 0;
            state.enc_vel_reset = 1;
            state.theta_ref = 0.0f;
            LOG_WARN("FELL — disarmed (eff=%.1f deg)", eff_angle);
        }
        else if (!state.armed && state.trying && eff_angle < 10.0f)
        {
            // Back within range — re-arm automatically
            state.armed = 1;
            motor_hal_standby(0);
            rc_led_set(RC_LED_GREEN, 1);
            LOG_INFO("RECOVERED — re-armed (eff=%.1f deg)", eff_angle);
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
            if (g_debug_config.controllers.D2_drive && state.armed)
            {
                static float last_correction = 0.0f;

                // Reset accumulated correction on re-arm so stale last_correction
                // doesn't slam theta_ref immediately after a fall/recovery.
                if (!prev_armed_d2)
                    last_correction = 0.0f;
                float correction = 0.0f;

                // Use raw stick only — exclude D2's own correction so it doesn't
                // trick the hold logic into thinking the user is driving.
                bool stick_centered = (fabsf(raw_stick_ref) < 0.5f); /* 0.5 deg ~3% of MAX_THETA_REF */

                // ── D2 verbose debug (enable via IPC: {"type":"debug_d2","value":true}) ──
                static uint64_t d2_log_last_us = 0;
                if (g_debug_config.debug_d2)
                {
                    uint64_t d2_now_us = rc_nanos_since_boot() / 1000;
                    if (d2_now_us - d2_log_last_us >= 500000) // 2 Hz
                    {
                        d2_log_last_us = d2_now_us;
                        int32_t d2_err = state.enc_pos - state.enc_pos_target;
                        printf("[D2] armed=%d D2_en=%d stick_cen=%d raw_stick=%.3f\n"
                               "     enc_pos=%d enc_pos_target=%d err=%d\n"
                               "     enc_vel=%d stopped_vel=%d back_to_spot=%d\n"
                               "     last_corr=%.4f theta_ref=%.4f\n",
                               state.armed,
                               g_debug_config.controllers.D2_drive,
                               (int)stick_centered,
                               raw_stick_ref,
                               state.enc_pos,
                               state.enc_pos_target,
                               d2_err,
                               state.enc_velocity,
                               g_pos_config.stopped_vel,
                               g_pos_config.back_to_spot,
                               last_correction,
                               state.theta_ref);
                        fflush(stdout);
                    }
                }

                if (stick_centered)
                {
                    int32_t err = state.enc_pos_target - state.enc_pos;
                    int32_t absErr = abs(err);

                    if (absErr < 2)
                    {
                        // Inside tight deadband — zero and skip zone logic entirely
                        last_correction = 0.0f;
                        correction = 0.0f;
                    }
                    else if (g_pos_config.back_to_spot)
                    {
                        // Full zone-based proportional hold
                        if (absErr > g_pos_config.zone_a)
                            correction = (float)err / g_pos_config.scale_a;
                        else if (absErr > g_pos_config.zone_b)
                            correction = (float)err / g_pos_config.scale_b;
                        else if (absErr > g_pos_config.zone_c)
                            correction = (float)err / g_pos_config.scale_c;
                        else
                            correction = (float)err / g_pos_config.scale_d;
                    }
                    else
                    {
                        // Loose hold: only correct inside zone_c, otherwise drift free.
                        // err = target - pos, positive err means bot is behind target,
                        // so positive correction (lean forward) is correct.
                        if (absErr < g_pos_config.zone_c)
                            correction = (float)err / g_pos_config.scale_d;
                        else
                            state.enc_pos_target = state.enc_pos;
                    }

                    // Velocity damping — resist oscillation around target
                    float vel_damp = (float)state.enc_velocity / g_pos_config.vel_scale_stop;
                    state.d2_pos_correction = correction;
                    state.d2_vel_damp = -vel_damp;
                    if (absErr > 5)
                        correction -= vel_damp;
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
                }

                // Rate-limit correction (reference implementation: don't change restAngle by more
                // than max_angle_rate per loop tick — prevents slamming theta_ref)
                float delta = correction - last_correction;
                if (delta > g_pos_config.max_angle_rate)
                    delta = g_pos_config.max_angle_rate;
                if (delta < -g_pos_config.max_angle_rate)
                    delta = -g_pos_config.max_angle_rate;
                correction = last_correction + delta;
                last_correction = correction;

                // Hard clamp
                if (correction > g_pos_config.max_correction)
                    correction = g_pos_config.max_correction;
                if (correction < -g_pos_config.max_correction)
                    correction = -g_pos_config.max_correction;

                state.d2_correction_out = correction;
                state.theta_ref += correction;
                // Clamp total lean D2 can command — not just per-tick step
                if (state.theta_ref >  g_pos_config.max_correction)
                    state.theta_ref =  g_pos_config.max_correction;
                if (state.theta_ref < -g_pos_config.max_correction)
                    state.theta_ref = -g_pos_config.max_correction;
            }
            else
            {
                // D2 disabled — keep target synced so it's ready when re-enabled
                if (g_debug_config.debug_d2)
                {
                    static uint64_t d2_else_last_us = 0;
                    uint64_t d2_now_us = rc_nanos_since_boot() / 1000;
                    if (d2_now_us - d2_else_last_us >= 500000)
                    {
                        d2_else_last_us = d2_now_us;
                        printf("[D2-ELSE] D2_drive=%d armed=%d  => enc_pos_target being synced to %d\n",
                               g_debug_config.controllers.D2_drive,
                               state.armed,
                               state.enc_pos);
                        fflush(stdout);
                    }
                }
                state.enc_pos_target = state.enc_pos;
                state.pos_setpoint = state.pos;
                if (!state.armed)
                    pid_reset(&drive_pid);
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
                int32_t d2_err = state.enc_pos_target - state.enc_pos;
                float phi_diff = (state.phi_left - state.phi_right) / 2.0f;
                printf("[D2] en=%d  pos=%d  tgt=%d  err=%d  vel=%d  corr_out=%.3f  theta_ref=%.3f\n",
                       g_debug_config.controllers.D2_drive,
                       state.enc_pos, state.enc_pos_target, d2_err,
                       state.enc_velocity,
                       state.d2_correction_out,
                       state.theta_ref);
                printf("[D3] en=%d  psi=%.2f  phi_diff=%.2f  steering=%.3f  latch=%.3f  latched=%d\n",
                       g_debug_config.controllers.D3_steering,
                       state.psi, phi_diff,
                       state.steering, state.steering_latch,
                       state.steering_latched);
                fflush(stdout);
            }
        }

        // Saturate references before the next ISR reads them
        rc_saturate_float(&state.theta_ref, -MAX_THETA_REF, MAX_THETA_REF);
        rc_saturate_float(&state.steering, -MAX_STEERING, MAX_STEERING);

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