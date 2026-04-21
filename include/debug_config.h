/**
 * @file debug_config.h
 * @brief Debug configuration and telemetry structures for balance_bot
 * 
 * This file defines all debug flags, telemetry data structures, and
 * configuration options that can be controlled from the iPhone app.
 */

#ifndef DEBUG_CONFIG_H
#define DEBUG_CONFIG_H

#include "rc_compat.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// TELEMETRY DATA STRUCTURES
// ============================================================================

/**
 * @brief Encoder telemetry data
 */
typedef struct {
    int32_t left_ticks;      // Raw encoder ticks
    int32_t right_ticks;
    float left_rad;          // Position in radians
    float right_rad;
    float left_vel;          // Velocity in rad/s
    float right_vel;
} encoder_telemetry_t;

/**
 * @brief IMU telemetry data (full state)
 */
typedef struct {
    // Attitude (orientation) - transformed via imu_config, used by PID
    float theta;             // Pitch - forward/back lean (rad)
    float phi;               // Roll - side lean (rad)
    float psi;               // Yaw - rotation (rad)
    
    // Angular rates
    float theta_dot;         // Pitch rate (rad/s)
    float phi_dot;           // Roll rate (rad/s)
    float psi_dot;           // Yaw rate (rad/s)

    // Raw quaternion from DMP - no gimbal lock, used for 3D visualization
    float qw, qx, qy, qz;

    // Raw accelerometer (m/s²)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // Raw gyroscope (rad/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_telemetry_t;

/**
 * @brief PID controller state (for telemetry)
 */
typedef struct {
    bool enabled;            // Is this controller active?
    float setpoint;          // Desired value
    float measurement;       // Actual value
    float error;             // setpoint - measurement
    float p_term;            // Proportional term
    float i_term;            // Integral term
    float d_term;            // Derivative term
    float output;            // Final controller output
    float kp, ki, kd;        // Current gains (for app sync on connect)
} pid_telemetry_t;

/**
 * @brief Motor command telemetry
 */
typedef struct {
    float left_duty;         // Left motor duty cycle (-1 to 1)
    float right_duty;        // Right motor duty cycle (-1 to 1)
} motor_telemetry_t;

/**
 * @brief External UART input telemetry
 */
typedef struct {
    bool valid;              // Is data fresh?
    float x;                 // Lateral command (-1 to 1)
    float y;                 // Forward command (-1 to 1)
    float confidence;        // Source confidence (0 to 1)
} ext_input_telemetry_t;

/**
 * @brief System status telemetry
 */
typedef enum {
    BATT_UNKNOWN  = 0,
    BATT_OK       = 1,
    BATT_WARNING  = 2,
    BATT_CRITICAL = 3,
} batt_status_t;

typedef struct {
    float battery_voltage;   // Battery voltage (V) — onboard ADC (legacy)
    bool armed;              // Are motors armed?
    int mode;                // 0=balance, 1=ext_input, 2=manual
    float loop_hz;           // Actual control loop frequency
    uint32_t uptime_sec;     // Seconds since start
    float theta_offset;      // Balance point trim (deg)
    float batt_voltage;      // External 3S voltage from batt_monitor (V), -1 if unavailable
    batt_status_t batt_status; // ok / warning / critical / unknown
} system_telemetry_t;

/**
 * @brief Complete telemetry packet
 * 
 * This structure contains all possible telemetry data.
 * Only enabled fields are populated and transmitted.
 */
typedef struct {
    uint64_t timestamp_us;          // Microsecond timestamp
    
    encoder_telemetry_t encoders;   // Encoder data
    imu_telemetry_t imu;             // IMU data
    
    pid_telemetry_t D1_balance;      // Balance PID state
    pid_telemetry_t D2_drive;        // Drive PID state
    pid_telemetry_t D3_steering;     // Steering PID state
    
    motor_telemetry_t motors;        // Motor commands
    ext_input_telemetry_t ext_input; // External UART input
    system_telemetry_t system;       // System status
} telemetry_data_t;

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

/**
 * @brief Telemetry enable flags
 * 
 * These flags control which telemetry data is collected and transmitted
 * to the iPhone app. Disabling unused telemetry saves bandwidth and CPU.
 */
typedef struct {
    bool encoders;           // Encoder ticks/position/velocity
    bool imu_attitude;       // Theta, phi, psi only (for 3D visualization)
    bool imu_full;           // All IMU data (accel, gyro) - HIGH BANDWIDTH
    bool pid_states;         // PID errors, terms, outputs
    bool motor_commands;     // Motor duty cycles
    bool ext_input;          // External UART input data
    bool system_status;      // Battery, armed, mode - always recommended
} telemetry_enables_t;

/**
 * @brief Video overlay configuration
 *
 * Flags forwarded to an external coprocessor (e.g. RPi) to control
 * what is overlaid on any video stream it produces.
 */
typedef struct {
    bool crosshair;          // Draw crosshair / target marker
    bool stats;              // Show FPS / detection stats
} video_overlays_t;

/**
 * @brief Telemetry update rates (Hz)
 */
typedef struct {
    int system_status;       // System status update rate (default: 1 Hz)
    int pid_states;          // PID state update rate (default: 10 Hz)
    int full_telemetry;      // Full telemetry rate (default: 10 Hz, max: 100 Hz)
} telemetry_rates_t;

/**
 * @brief Logging configuration
 */
typedef enum {
    LOG_LEVEL_DEBUG,         // Everything
    LOG_LEVEL_INFO,          // Normal operation
    LOG_LEVEL_WARN,          // Warnings only
    LOG_LEVEL_ERROR          // Errors only
} log_level_t;

typedef struct {
    log_level_t level;       // Current log level
    bool console;            // Print to console
    bool file;               // Write to log file
    bool timestamps;         // Include timestamps
} logging_config_t;

/**
 * @brief Console display blocks
 *
 * Controls which data panels are printed to the terminal at runtime.
 * Each block prints on its own update cadence (every N main-loop ticks).
 * Set a block's interval to 0 to disable it entirely.
 *
 * Intervals are in main-loop ticks (100 Hz → 100 ticks = 1 s).
 *
 *   SBUS TX   — live channel values, switch states, failsafe flags
 *   PID       — setpoint / measurement / P / I / D / output for each controller
 *   Encoders  — raw ticks, position (rad), velocity (rad/s)
 *   IMU       — pitch / roll / yaw + rates
 *   Motors    — left / right duty cycle
 *   System    — armed state, mode, battery voltage, loop Hz
 */
typedef struct {
    int sbus_tx;        /**< SBUS TX channel display   (0 = off, else ticks) */
    int pid;            /**< PID state display          (0 = off, else ticks) */
    int encoders;       /**< Encoder display            (0 = off, else ticks) */
    int imu;            /**< IMU display                (0 = off, else ticks) */
    int motors;         /**< Motor duty display         (0 = off, else ticks) */
    int system;         /**< System status display      (0 = off, else ticks) */
} display_config_t;

/**
 * @brief Complete debug configuration
 *
 * This is the master configuration structure that controls all
 * debug features, telemetry, and logging.
 */
typedef struct {
    telemetry_enables_t telemetry;      // What telemetry to send
    video_overlays_t overlays;          // Video overlay settings
    bool debug_d2;                      // Verbose D2 position controller logging
    telemetry_rates_t rates;            // Update rates
    logging_config_t logging;           // Logging configuration
    display_config_t display;           // Console display blocks
} debug_config_t;

// ============================================================================
// GLOBAL DEBUG CONFIGURATION
// ============================================================================

extern debug_config_t g_debug_config;
extern telemetry_data_t g_telemetry_data;

// ============================================================================
// LOGGING MACROS (FIXED FORMAT SPECIFIERS)
// ============================================================================

/**
 * Conditional logging based on current log level
 */
#define LOG_DEBUG(fmt, ...) \
    do { \
        if (g_debug_config.logging.level <= LOG_LEVEL_DEBUG) { \
            if (g_debug_config.logging.timestamps) { \
                printf("[DEBUG][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot()/1000000), ##__VA_ARGS__); \
            } else { \
                printf("[DEBUG] " fmt "\n", ##__VA_ARGS__); \
            } \
        } \
    } while(0)

#define LOG_INFO(fmt, ...) \
    do { \
        if (g_debug_config.logging.level <= LOG_LEVEL_INFO) { \
            if (g_debug_config.logging.timestamps) { \
                printf("[INFO][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot()/1000000), ##__VA_ARGS__); \
            } else { \
                printf("[INFO] " fmt "\n", ##__VA_ARGS__); \
            } \
        } \
    } while(0)

#define LOG_WARN(fmt, ...) \
    do { \
        if (g_debug_config.logging.level <= LOG_LEVEL_WARN) { \
            if (g_debug_config.logging.timestamps) { \
                fprintf(stderr, "[WARN][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot()/1000000), ##__VA_ARGS__); \
            } else { \
                fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__); \
            } \
        } \
    } while(0)

#define LOG_ERROR(fmt, ...) \
    do { \
        if (g_debug_config.logging.level <= LOG_LEVEL_ERROR) { \
            if (g_debug_config.logging.timestamps) { \
                fprintf(stderr, "[ERROR][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot()/1000000), ##__VA_ARGS__); \
            } else { \
                fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__); \
            } \
        } \
    } while(0)

// ============================================================================
// DEFAULT CONFIGURATIONS
// ============================================================================

/**
 * @brief Get default debug configuration
 * 
 * Returns a sensible default configuration suitable for most use cases.
 * Enables basic telemetry without overwhelming the network.
 */
static inline debug_config_t get_default_debug_config(void) {
    debug_config_t config = {
        .telemetry = {
            .encoders = false,
            .imu_attitude = true,      // For 3D visualization
            .imu_full = false,         // Disable high-bandwidth data
            .pid_states = true,
            .motor_commands = false,
            .ext_input = true,
            .system_status = true
        },
.overlays = {
            .crosshair = true,
            .stats = false
        },
        .rates = {
            .system_status = 1,        // 1 Hz
            .pid_states = 10,          // 10 Hz
            .full_telemetry = 10       // 10 Hz
        },
        .logging = {
            .level = LOG_LEVEL_INFO,
            .console = true,
            .file = false,
            .timestamps = true
        },
        .display = {
            .sbus_tx  = 0,      /* off by default — enable with -d sbus    */
            .pid      = 0,      /* off by default — enable with -d pid     */
            .encoders = 0,      /* off by default — enable with -d enc     */
            .imu      = 0,      /* off by default — enable with -d imu     */
            .motors   = 0,      /* off by default — enable with -d mot     */
            .system   = 100,    /* system status on by default  (~1 Hz)    */
        }
    };
    return config;
}

#endif // DEBUG_CONFIG_H
