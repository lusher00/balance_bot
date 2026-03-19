/**
 * @file robot.c
 * @brief Main robot control loop with telemetry and IPC integration
 */

#include "balance_bot.h"
#include "motor_hal.h"
#include "display.h"
#include <stdio.h>
#include <math.h>
#include <robotcontrol.h>

// Global state
robot_state_t state = {0};
rc_mpu_data_t mpu_data;
pid_controller_t balance_pid, drive_pid, steering_pid;

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
 * @brief Pause button — cycle operating mode
 */
static void on_pause_press(void)
{
    switch (state.mode)
    {
    case MODE_BALANCE:
        state.mode = MODE_EXT_INPUT;
        LOG_INFO("Mode: EXT INPUT");
        rc_led_set(RC_LED_RED, 1);
        break;
    case MODE_EXT_INPUT:
        state.mode = MODE_BALANCE;
        LOG_INFO("Mode: BALANCE");
        rc_led_set(RC_LED_RED, 0);
        break;
    default:
        state.mode = MODE_BALANCE;
        break;
    }
}

/**
 * @brief Mode button release — toggle arm/disarm
 */
static void on_mode_release(void)
{
    state.armed = !state.armed;

    if (state.armed)
    {
        LOG_INFO("ARMED");
        rc_led_set(RC_LED_GREEN, 1);
        motor_hal_standby(0);
    }
    else
    {
        LOG_INFO("DISARMED");
        rc_led_set(RC_LED_GREEN, 0);
        motor_hal_standby(1);
        motor_hal_set_both(0.0f, 0.0f);
        pid_reset(&balance_pid);
        pid_reset(&drive_pid);
        pid_reset(&steering_pid);
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

    if (rc_adc_init() < 0)
        LOG_WARN("ADC init failed — battery readings unavailable");

    // PIDs
    pid_init(&balance_pid, BALANCE_KP, BALANCE_KI, BALANCE_KD, DT);
    pid_init(&drive_pid, DRIVE_KP, DRIVE_KI, DRIVE_KD, DT);
    pid_init(&steering_pid, STEERING_KP, STEERING_KI, STEERING_KD, DT);
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

    // Buttons
    LOG_INFO("Initializing buttons...");
    if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
                       RC_BTN_DEBOUNCE_DEFAULT_US) == -1)
    {
        fprintf(stderr, "ERROR: Pause button init failed\n");
        return -1;
    }
    if (rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
                       RC_BTN_DEBOUNCE_DEFAULT_US) == -1)
    {
        fprintf(stderr, "ERROR: Mode button init failed\n");
        return -1;
    }
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, NULL);
    rc_button_set_callbacks(RC_BTN_PIN_MODE, NULL, on_mode_release);

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

        // ── Global wheel position ─────────────────────────────────────────
        state.pos = (state.phi_left + state.phi_right) / 2.0f + state.theta;

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

        // ── SBUS arming / drive commands ──────────────────────────────────
        // state.trying  — operator wants the robot to balance (arm switch)
        // state.armed   — actually driving motors (cleared on fall, restored when upright)
        if (sbus_is_connected())
        {
            int kill = sbus_get_kill();
            int arm = sbus_get_arm();

            if (kill < 2)
            {
                if (state.trying)
                {
                    state.trying = 0;
                    state.armed = 0;
                    rc_led_set(RC_LED_GREEN, 0);
                    LOG_WARN("SBUS kill switch — DISARMED");
                }
            }

            static int prev_sbus_arm = 0;
            if (arm == 2 && prev_sbus_arm < 2 && kill == 2)
            {
                if (fabsf(state.theta - state.theta_offset) > 14.0f)
                {
                    LOG_WARN("SBUS ARM REJECTED — angle too large (%.1f deg)",
                             state.theta - state.theta_offset);
                }
                else
                {
                    state.trying = 1;
                    state.armed = 1;
                    motor_hal_standby(0);
                    rc_led_set(RC_LED_GREEN, 1);
                    LOG_INFO("SBUS ARMED (theta=%.2f deg)", state.theta);
                }
            }
            else if (arm < 2 && prev_sbus_arm == 2)
            {
                state.trying = 0;
                state.armed = 0;
                rc_led_set(RC_LED_GREEN, 0);
                LOG_INFO("SBUS DISARMED");
            }
            prev_sbus_arm = arm;

            if (state.armed && state.mode == MODE_BALANCE)
            {
                state.theta_ref = sbus_get_drive() * MAX_THETA_REF;
                state.steering = sbus_get_turn() * MAX_STEERING;
            }
        }

        // ── Fall detection / auto-recovery ───────────────────────────────
        float eff_angle = fabsf(state.theta - state.theta_offset);
        if (state.armed && eff_angle > 15.0f)
        {
            // Fell — cut motors but stay trying if operator hasn't disarmed
            state.armed = 0;
            motor_hal_standby(1);
            rc_led_set(RC_LED_GREEN, 0);
            LOG_WARN("FELL — auto-disarmed (eff=%.1f deg)", eff_angle);
        }
        else if (!state.armed && state.trying && eff_angle < 10.0f)
        {
            // Back within range — re-arm
            state.armed = 1;
            motor_hal_standby(0);
            rc_led_set(RC_LED_GREEN, 1);
            LOG_INFO("RECOVERED — re-armed (eff=%.1f deg)", eff_angle);
        }

        // ── D2 position (drive) controller ───────────────────────────────
        if (g_debug_config.controllers.D2_drive && state.armed)
        {
            state.pos_setpoint += state.theta_ref * DT;

            float d2_out = pid_update(&drive_pid, state.pos_setpoint, state.pos);
            rc_saturate_float(&d2_out, -MAX_THETA_REF, MAX_THETA_REF);
            state.theta_ref = -d2_out;
        }
        else
        {
            state.pos_setpoint = state.pos;
            if (!state.armed)
                pid_reset(&drive_pid);
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
