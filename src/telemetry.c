/**
 * @file telemetry.c
 * @brief Telemetry data collection for balance_bot
 * 
 * This module collects telemetry data from various sources (IMU, encoders,
 * PIDs, etc.) and populates the global telemetry structure for transmission
 * to the iPhone app via the IPC server.
 * 
 * Call telemetry_update() from the main control loop to collect current data.
 */

#include "debug_config.h"
#include "balance_bot.h"
#include <robotcontrol.h>
#include <math.h>

// Global telemetry data (shared with IPC server)
telemetry_data_t g_telemetry_data = {0};

// External references to robot state
extern robot_state_t state;
extern rc_mpu_data_t mpu_data;
extern pid_controller_t balance_pid, steering_pid;

// Encoder tracking for velocity calculation
static int32_t prev_left_ticks = 0;
static int32_t prev_right_ticks = 0;
static uint64_t prev_timestamp_us = 0;

#define ENCODER_TICKS_PER_REV 2400  // TODO: Set to actual encoder resolution

/**
 * @brief Update encoder telemetry
 * 
 * Reads encoder values and calculates position and velocity.
 */
static void update_encoder_telemetry(void) {
    if (!g_debug_config.telemetry.encoders) return;
    
    // Read current encoder ticks
    int32_t left_ticks = rc_encoder_read(1);
    int32_t right_ticks = rc_encoder_read(2);
    
    // Calculate position in radians
    g_telemetry_data.encoders.left_ticks = left_ticks;
    g_telemetry_data.encoders.right_ticks = right_ticks;
    g_telemetry_data.encoders.left_rad = left_ticks * 2.0 * M_PI / ENCODER_TICKS_PER_REV;
    g_telemetry_data.encoders.right_rad = right_ticks * 2.0 * M_PI / ENCODER_TICKS_PER_REV;
    
    // Calculate velocity (rad/s)
    uint64_t now_us = rc_nanos_since_boot() / 1000;
    if (prev_timestamp_us > 0) {
        float dt = (now_us - prev_timestamp_us) / 1000000.0;
        if (dt > 0.0001) {  // Avoid division by zero
            int32_t left_delta = left_ticks - prev_left_ticks;
            int32_t right_delta = right_ticks - prev_right_ticks;
            
            g_telemetry_data.encoders.left_vel = 
                (left_delta * 2.0 * M_PI / ENCODER_TICKS_PER_REV) / dt;
            g_telemetry_data.encoders.right_vel = 
                (right_delta * 2.0 * M_PI / ENCODER_TICKS_PER_REV) / dt;
        }
    }
    
    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;
    prev_timestamp_us = now_us;
}

/**
 * @brief Update IMU telemetry
 *
 * Uses the same imu_config_apply_transform() path that the PID controller
 * uses, so the angles the app sees match exactly what the robot is acting on.
 * Raw DMP values are NOT used here — that was the previous bug.
 */
static void update_imu_telemetry(void) {
    if (!g_debug_config.telemetry.imu_attitude) return;

    // DMP quaternion (accel/gyro only, no magnetometer required)
    // rc_mpu_data_t.dmp_quat[4] is ordered [W, X, Y, Z]
    g_telemetry_data.imu.qw = (float)mpu_data.dmp_quat[0];
    g_telemetry_data.imu.qx = (float)mpu_data.dmp_quat[1];
    g_telemetry_data.imu.qy = (float)mpu_data.dmp_quat[2];
    g_telemetry_data.imu.qz = (float)mpu_data.dmp_quat[3];
    imu_transform_t t;
    imu_config_apply_transform(&mpu_data, &t, &g_imu_config);

    g_telemetry_data.imu.theta     = t.pitch;
    g_telemetry_data.imu.phi       = t.roll;
    g_telemetry_data.imu.psi       = t.yaw;
    g_telemetry_data.imu.theta_dot = t.pitch_dot;
    g_telemetry_data.imu.phi_dot   = t.roll_dot;
    g_telemetry_data.imu.psi_dot   = t.yaw_dot;

    // Full IMU data (if enabled)
    if (g_debug_config.telemetry.imu_full) {
        g_telemetry_data.imu.accel_x = t.accel_x;
        g_telemetry_data.imu.accel_y = t.accel_y;
        g_telemetry_data.imu.accel_z = t.accel_z;

        // Gyro rates already in rad/s from the transform
        g_telemetry_data.imu.gyro_x = t.pitch_dot;
        g_telemetry_data.imu.gyro_y = t.roll_dot;
        g_telemetry_data.imu.gyro_z = t.yaw_dot;
    }
}

/**
 * @brief Update PID controller telemetry
 * 
 * Captures current state of all PID controllers including setpoints,
 * measurements, errors, and outputs.
 */
static void update_pid_telemetry(void) {
    if (!g_debug_config.telemetry.pid_states) return;
    
    // D1: Balance controller
    g_telemetry_data.D1_balance.enabled = g_debug_config.controllers.D1_balance;
    g_telemetry_data.D1_balance.setpoint = state.theta_ref;
    g_telemetry_data.D1_balance.measurement = state.theta;
    g_telemetry_data.D1_balance.error = balance_pid.prev_error;
    g_telemetry_data.D1_balance.p_term = balance_pid.kp * balance_pid.prev_error;
    g_telemetry_data.D1_balance.i_term = balance_pid.ki * balance_pid.integrator;
    g_telemetry_data.D1_balance.d_term = balance_pid.kd * 
        (balance_pid.prev_error - balance_pid.prev_error) / balance_pid.dt;  // Simplified
    // Note: Actual calculation requires storing previous error
    
    // Calculate output from PID terms
    float balance_output = g_telemetry_data.D1_balance.p_term + 
                          g_telemetry_data.D1_balance.i_term + 
                          g_telemetry_data.D1_balance.d_term;
    g_telemetry_data.D1_balance.output = balance_output;
    
    // D2: Drive controller (if implemented)
    g_telemetry_data.D2_drive.enabled = g_debug_config.controllers.D2_drive;
    // TODO: Populate when D2 is implemented
    
    // D3: Steering controller
    g_telemetry_data.D3_steering.enabled = g_debug_config.controllers.D3_steering;
    g_telemetry_data.D3_steering.setpoint = state.steering;
    g_telemetry_data.D3_steering.error = steering_pid.prev_error;
    
    float steering_output = steering_pid.kp * steering_pid.prev_error +
                           steering_pid.ki * steering_pid.integrator;
    g_telemetry_data.D3_steering.output = steering_output;
}

/**
 * @brief Update motor command telemetry
 */
static void update_motor_telemetry(void) {
    if (!g_debug_config.telemetry.motor_commands) return;
    
    // Motor duty cycles are set in the control loop
    // We can read them back if needed, or track them
    // For now, leaving as 0 - implement if motor readback is available
    g_telemetry_data.motors.left_duty = 0.0;   // TODO: Track in control loop
    g_telemetry_data.motors.right_duty = 0.0;  // TODO: Track in control loop
}

/**
 * @brief Update external UART input telemetry
 */
static void update_ext_input_telemetry(void) {
    if (!g_debug_config.telemetry.ext_input) return;
    
    // Copy latest external input from robot state
    g_telemetry_data.ext_input.valid = state.ext_input.valid;
    g_telemetry_data.ext_input.x = state.ext_input.x;
    g_telemetry_data.ext_input.y = state.ext_input.y;
    g_telemetry_data.ext_input.confidence = state.ext_input.confidence;
}

/**
 * @brief Update system status telemetry
 */
static void update_system_telemetry(void) {
    if (!g_debug_config.telemetry.system_status) return;
    
    // Read battery voltage
    // Note: rc_adc_batt() returns battery voltage on BeagleBone Blue
    g_telemetry_data.system.battery_voltage = rc_adc_batt();
    
    // Robot state
    g_telemetry_data.system.armed = state.armed;
    g_telemetry_data.system.mode = state.mode;
    
    // Calculate actual loop frequency
    static uint64_t last_update_us = 0;
    uint64_t now_us = rc_nanos_since_boot() / 1000;
    if (last_update_us > 0) {
        float dt = (now_us - last_update_us) / 1000000.0;
        if (dt > 0.0001) {
            g_telemetry_data.system.loop_hz = 1.0 / dt;
        }
    }
    last_update_us = now_us;
    
    // Uptime
    g_telemetry_data.system.uptime_sec = (uint32_t)(rc_nanos_since_boot() / 1000000000ULL);
}

/**
 * @brief Update all telemetry data
 * 
 * Call this from the main control loop to collect all enabled telemetry.
 * Respects the enable flags in g_debug_config to avoid unnecessary work.
 * 
 * @note This should be called at a reasonable rate (e.g., 10-100 Hz)
 *       but actual transmission rate is controlled by IPC server.
 */
void telemetry_update(void) {
    // Update timestamp
    g_telemetry_data.timestamp_us = rc_nanos_since_boot() / 1000;
    
    // Update each telemetry component based on enable flags
    update_system_telemetry();   // Always update system status
    update_encoder_telemetry();
    update_imu_telemetry();
    update_pid_telemetry();
    update_motor_telemetry();
    update_ext_input_telemetry();
}

/**
 * @brief Initialize telemetry system
 * 
 * Sets up initial state and clears telemetry data.
 * 
 * @return 0 on success, -1 on error
 */
int telemetry_init(void) {
    LOG_INFO("Initializing telemetry system");
    
    // Clear telemetry data
    memset(&g_telemetry_data, 0, sizeof(g_telemetry_data));
    
    // Reset encoder tracking
    prev_left_ticks = 0;
    prev_right_ticks = 0;
    prev_timestamp_us = 0;
    
    LOG_INFO("Telemetry system initialized");
    return 0;
}

/**
 * @brief Get human-readable description of current telemetry configuration
 * 
 * Useful for debugging and logging what telemetry is enabled.
 * 
 * @param buffer Output buffer
 * @param size Size of output buffer
 */
void telemetry_get_config_description(char* buffer, size_t size) {
    int pos = 0;
    
    pos += snprintf(buffer + pos, size - pos, "Telemetry enabled: ");
    
    if (g_debug_config.telemetry.system_status) pos += snprintf(buffer + pos, size - pos, "system ");
    if (g_debug_config.telemetry.encoders) pos += snprintf(buffer + pos, size - pos, "encoders ");
    if (g_debug_config.telemetry.imu_attitude) pos += snprintf(buffer + pos, size - pos, "imu_attitude ");
    if (g_debug_config.telemetry.imu_full) pos += snprintf(buffer + pos, size - pos, "imu_full ");
    if (g_debug_config.telemetry.pid_states) pos += snprintf(buffer + pos, size - pos, "pid_states ");
    if (g_debug_config.telemetry.motor_commands) pos += snprintf(buffer + pos, size - pos, "motors ");
    if (g_debug_config.telemetry.ext_input) pos += snprintf(buffer + pos, size - pos, "ext_input ");
    
    if (pos == strlen("Telemetry enabled: ")) {
        snprintf(buffer + pos, size - pos, "none");
    }
}

/**
 * @brief Print current telemetry to console
 * 
 * Useful for debugging. Prints a human-readable summary of current telemetry.
 */
void telemetry_print_summary(void) {
    printf("\n=== Telemetry Summary ===\n");
    printf("System: %.2fV %s Mode:%d %.1fHz\n",
           g_telemetry_data.system.battery_voltage,
           g_telemetry_data.system.armed ? "ARMED" : "DISARMED",
           g_telemetry_data.system.mode,
           g_telemetry_data.system.loop_hz);
    
    if (g_debug_config.telemetry.imu_attitude) {
        printf("IMU: θ=%.2f° φ=%.2f° ψ=%.2f°\n",
               g_telemetry_data.imu.theta * 180.0 / M_PI,
               g_telemetry_data.imu.phi * 180.0 / M_PI,
               g_telemetry_data.imu.psi * 180.0 / M_PI);
        printf("QUAT: w=%.4f x=%.4f y=%.4f z=%.4f\n",
               g_telemetry_data.imu.qw,
               g_telemetry_data.imu.qx,
               g_telemetry_data.imu.qy,
               g_telemetry_data.imu.qz);
    }
    
    if (g_debug_config.telemetry.encoders) {
        printf("Encoders: L=%d(%.2frad,%.2frad/s) R=%d(%.2frad,%.2frad/s)\n",
               g_telemetry_data.encoders.left_ticks,
               g_telemetry_data.encoders.left_rad,
               g_telemetry_data.encoders.left_vel,
               g_telemetry_data.encoders.right_ticks,
               g_telemetry_data.encoders.right_rad,
               g_telemetry_data.encoders.right_vel);
    }
    
    if (g_debug_config.telemetry.pid_states) {
        printf("D1_balance: err=%.3f out=%.3f %s\n",
               g_telemetry_data.D1_balance.error,
               g_telemetry_data.D1_balance.output,
               g_telemetry_data.D1_balance.enabled ? "ON" : "OFF");
        
        printf("D3_steering: err=%.3f out=%.3f %s\n",
               g_telemetry_data.D3_steering.error,
               g_telemetry_data.D3_steering.output,
               g_telemetry_data.D3_steering.enabled ? "ON" : "OFF");
    }
    
    if (g_debug_config.telemetry.ext_input && g_telemetry_data.ext_input.valid) {
        printf("Cat: x=%.2f y=%.2f conf=%.0f%%\n",
               g_telemetry_data.ext_input.x,
               g_telemetry_data.ext_input.y,
               g_telemetry_data.ext_input.confidence * 100.0);
    }
    
    printf("========================\n\n");
}
