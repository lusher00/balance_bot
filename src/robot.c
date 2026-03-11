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
pid_controller_t balance_pid, steering_pid;

// Debug configuration (defined in debug_config.h, initialized in main.c)
debug_config_t g_debug_config;

// Telemetry counters
static uint64_t telemetry_counter = 0;
static uint64_t last_telemetry_broadcast = 0;

// Motor duty tracking (for telemetry)
static float last_left_duty = 0.0f;
static float last_right_duty = 0.0f;

// Motor output handoff: ISR writes these, main loop applies them to hardware.
// volatile so the compiler doesn't optimize away cross-context reads/writes.
static volatile float pending_left_duty  = 0.0f;
static volatile float pending_right_duty = 0.0f;
static volatile int   motor_output_ready = 0;

/**
 * @brief IMU interrupt callback (runs at SAMPLE_RATE_HZ)
 */
static void imu_interrupt(void) {
    // Apply mounting transform to get pitch/yaw in robot frame.
    // mpu_data is already populated by the DMP before this callback fires —
    // no additional I2C reads needed here.
    imu_transform_t imu_transformed;
    imu_config_apply_transform(&mpu_data, &imu_transformed, &g_imu_config);

    state.theta     = imu_transformed.pitch;      // deg
    state.theta_dot = imu_transformed.pitch_dot;  // deg/s
    state.psi       = imu_transformed.yaw;        // deg

    if (!state.armed) return;

    // PID — references and encoder positions are maintained by the main loop.
    float balance_output = 0.0f;
    float steering_output = 0.0f;

    if (g_debug_config.controllers.D1_balance) {
        balance_output = pid_update(&balance_pid,
                                    state.theta_ref + state.theta_offset,
                                    state.theta);
    }

    if (g_debug_config.controllers.D3_steering) {
        float phi_diff = (state.phi_left - state.phi_right) / 2.0f;
        steering_output = pid_update(&steering_pid, state.steering, phi_diff);
    } else {
        steering_output = state.steering;
    }

    float left_duty  = rc_saturate_float(&(float){balance_output - steering_output}, -1.0f, 1.0f);
    float right_duty = rc_saturate_float(&(float){balance_output + steering_output}, -1.0f, 1.0f);

    // Hand off to main loop — motor write happens there, not in the ISR.
    pending_left_duty  = left_duty;
    pending_right_duty = right_duty;
    motor_output_ready = 1;
}

/**
 * @brief Pause button callback
 */
static void on_pause_press(void) {
    // Cycle through modes
    switch (state.mode) {
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
 * @brief Mode button callback (arm/disarm)
 */
static void on_mode_release(void) {
    state.armed = !state.armed;
    
    if (state.armed) {
        LOG_INFO("ARMED");
        rc_led_set(RC_LED_GREEN, 1);
        motor_hal_standby(0);
    } else {
        LOG_INFO("DISARMED");
        rc_led_set(RC_LED_GREEN, 0);
        motor_hal_standby(1);
        motor_hal_set_both(0.0f, 0.0f);
        
        // Reset PIDs
        pid_reset(&balance_pid);
        pid_reset(&steering_pid);
    }
}

/**
 * @brief Initialize robot hardware and control system
 */
int robot_init(void) {
    // Kill any existing process
    if (rc_kill_existing_process(2.0) < -2) {
        return -1;
    }
    
    // Enable signal handler
    if (rc_enable_signal_handler() == -1) {
        fprintf(stderr, "ERROR: Failed to start signal handler\n");
        return -1;
    }

// Initialize ADC for battery reading
if (rc_adc_init() < 0) {
    LOG_WARN("ADC init failed - battery readings unavailable");
}
    
    // Initialize PIDs
    pid_init(&balance_pid, BALANCE_KP, BALANCE_KI, BALANCE_KD, DT);
    pid_init(&steering_pid, STEERING_KP, STEERING_KI, STEERING_KD, DT);

imu_config_load(&g_imu_config);
imu_config_print(&g_imu_config);    
    LOG_INFO("PID Controllers:");
    LOG_INFO("  D1_balance: Kp=%.1f Ki=%.1f Kd=%.1f", BALANCE_KP, BALANCE_KI, BALANCE_KD);
    LOG_INFO("  D3_steering: Kp=%.1f Ki=%.1f Kd=%.1f", STEERING_KP, STEERING_KI, STEERING_KD);
    
    // Initialize IMU
    LOG_INFO("Initializing IMU...");
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
    mpu_config.dmp_fetch_accel_gyro = 1;
    
    if (rc_mpu_initialize_dmp(&mpu_data, mpu_config) < 0) {
        fprintf(stderr, "ERROR: IMU initialization failed\n");
        return -1;
    }
    rc_mpu_set_dmp_callback(&imu_interrupt);
    
    // Motor HAL is initialised by main() before robot_init() is called
    
    // Initialize buttons
    LOG_INFO("Initializing buttons...");
    if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, 
                       RC_BTN_DEBOUNCE_DEFAULT_US) == -1) {
        fprintf(stderr, "ERROR: Pause button init failed\n");
        return -1;
    }
    if (rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
                       RC_BTN_DEBOUNCE_DEFAULT_US) == -1) {
        fprintf(stderr, "ERROR: Mode button init failed\n");
        return -1;
    }
    
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, NULL);
    rc_button_set_callbacks(RC_BTN_PIN_MODE, NULL, on_mode_release);
    
    // Initialize LEDs
    rc_led_set(RC_LED_GREEN, 0);
    rc_led_set(RC_LED_RED, 0);
    
    // Initialize robot state
    state.mode = MODE_BALANCE;
    state.armed = 0;
    state.theta_ref = 0.0f;
    state.theta_offset = 0.0f;
    state.steering = 0.0f;
    
    // Create PID file
    rc_make_pid_file();
    
    rc_set_state(RUNNING);
    
    LOG_INFO("Robot initialized successfully");
    return 0;
}

/**
 * @brief Main control loop
 */
void robot_run(void) {
    uint64_t loop_counter = 0;
    
    while (rc_get_state() != EXITING) {
        // ── Motor write (from last ISR PID output) ────────────────────────
        // Do this first to minimize latency from ISR to hardware.
        if (motor_output_ready) {
            float l = pending_left_duty;
            float r = pending_right_duty;
            motor_output_ready = 0;
            motor_hal_set_both(l, r);
            last_left_duty  = l;
            last_right_duty = r;
        }
        int left_ticks  = motor_hal_encoder_read(MOTOR_LEFT);
        int right_ticks = motor_hal_encoder_read(MOTOR_RIGHT);
        state.phi_left  = left_ticks  * (360.0f / ENCODER_TICKS_PER_REV);  // deg
        state.phi_right = right_ticks * (360.0f / ENCODER_TICKS_PER_REV);  // deg

        // ── Input sources → theta_ref / steering ─────────────────────────
        if (state.mode == MODE_EXT_INPUT) {
            input_packet_t pkt;
            if (uart_input_get(&pkt)) {
                state.ext_input = pkt;
                state.theta_ref = -pkt.y * MAX_THETA_REF;
                state.steering  =  pkt.x * MAX_STEERING;
            } else {
                state.theta_ref = 0.0f;
                state.steering  = 0.0f;
            }
        }

        xbox_update();
        sbus_update();

        // ── SBUS → robot state mapping ────────────────────────────────────
        if (sbus_is_connected()) {
            int kill = sbus_get_kill();   /* 0=low(kill) 1=mid(kill) 2=high(run) */
            int arm  = sbus_get_arm();    /* 0=low(disarm) 1=mid 2=high(armed)   */

            // Kill switch or failsafe: force immediate disarm if not fully high
            if (kill < 2 && state.armed) {
                state.armed = 0;
                motor_hal_standby(1);
                motor_hal_set_both(0.0f, 0.0f);
                pid_reset(&balance_pid);
                pid_reset(&steering_pid);
                rc_led_set(RC_LED_GREEN, 0);
                LOG_WARN("SBUS kill switch active — DISARMED");
            }

            // Arm on rising edge to pos 2; disarm on any drop below 2
            static int prev_sbus_arm = 0;
            if (arm == 2 && prev_sbus_arm < 2 && kill == 2) {
                // Reject arm if tilted too far
                if (fabsf(state.theta) > 14.0f) {
                    LOG_WARN("SBUS: ARM REJECTED — angle too large (%.1f deg)",
                             state.theta);
                } else {
                    state.armed = 1;
                    motor_hal_standby(0);
                    rc_led_set(RC_LED_GREEN, 1);
                    LOG_INFO("SBUS: ARMED (theta=%.2f deg)", state.theta);
                }
            } else if (arm < 2 && prev_sbus_arm == 2) {
                state.armed = 0;
                motor_hal_standby(1);
                motor_hal_set_both(0.0f, 0.0f);
                pid_reset(&balance_pid);
                pid_reset(&steering_pid);
                rc_led_set(RC_LED_GREEN, 0);
                LOG_INFO("SBUS: DISARMED");
            }
            prev_sbus_arm = arm;

            // Drive commands — sbus_get_drive/turn return 0 when kill/disarmed
            if (state.armed && state.mode == MODE_BALANCE) {
                state.theta_ref = sbus_get_drive() * MAX_THETA_REF;
                state.steering  = sbus_get_turn()  * MAX_STEERING;
            }
        }
        // ─────────────────────────────────────────────────────────────────

        // Saturate references (interrupt reads these, so do it here before next ISR)
        rc_saturate_float(&state.theta_ref, -MAX_THETA_REF, MAX_THETA_REF);
        rc_saturate_float(&state.steering,  -MAX_STEERING,   MAX_STEERING);

        // Update telemetry data
        telemetry_update();

        // Update motor duty in telemetry (tracked from IMU interrupt)
        g_telemetry_data.motors.left_duty = last_left_duty;
        g_telemetry_data.motors.right_duty = last_right_duty;
        
        // Broadcast telemetry at configured rate
        uint64_t now = rc_nanos_since_boot() / 1000000;  // Convert to ms
        uint64_t telemetry_interval = 1000 / g_debug_config.rates.pid_states;  // ms
        
        if (now - last_telemetry_broadcast >= telemetry_interval) {
            ipc_broadcast_telemetry();
            last_telemetry_broadcast = now;
            telemetry_counter++;
        }
        
        // Signal display thread to redraw
        display_update();
        
        loop_counter++;
        rc_usleep(10000);  // 10ms sleep (100 Hz main loop)
    }
}

/**
 * @brief Cleanup robot
 */
void robot_cleanup(void) {
    LOG_INFO("Cleaning up robot...");
    
    // Disarm and stop motors
    motor_hal_set_both(0.0f, 0.0f);
    motor_hal_cleanup();
    rc_mpu_power_off();
    
    // Turn off LEDs
    rc_led_set(RC_LED_GREEN, 0);
    rc_led_set(RC_LED_RED, 0);
    
    // Remove PID file
    rc_remove_pid_file();
    
    rc_set_state(EXITING);
    
    LOG_INFO("Robot cleanup complete");
}
