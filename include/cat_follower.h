/**
 * @file cat_follower.h
 * @brief Main header for balance_bot (formerly cat_follower)
 * 
 * This is the primary header file that includes all module interfaces
 * for the balance_bot self-balancing robot with iPhone app integration.
 * 
 * Architecture:
 * - D1 (balance): Angle controller - keeps robot upright
 * - D2 (drive): Position controller - drives via lean angle (optional)
 * - D3 (steering): Yaw controller - turns left/right
 * - Cat tracking: Optional input from RPi5 via UART
 * - IPC server: Communication with Node.js for iPhone app
 * - Telemetry: Real-time data broadcast to iPhone
 */

#ifndef CAT_FOLLOWER_H
#define CAT_FOLLOWER_H

#include <robotcontrol.h>
#include <stdbool.h>
#include <stdint.h>
#include "debug_config.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define SAMPLE_RATE_HZ      100
#define DT                  0.01

// UART for cat tracking (from RPi5)
#define CAT_UART_BUS        1
#define CAT_UART_BAUD       115200
#define CAT_TIMEOUT_MS      500

// Control limits
#define MAX_THETA_REF       0.3     // Max lean angle (rad) ~17°
#define MAX_STEERING        1.0     // Max steering command
#define CAT_STEERING_GAIN   0.5     // Cat following turn gain
#define CAT_APPROACH_GAIN   0.1     // Cat following forward gain
#define CAT_DEADBAND        0.15    // Center deadband (ignore if within ±15%)

// PID default gains (tunable via iPhone app)
#define BALANCE_KP 40.0
#define BALANCE_KI 0.0
#define BALANCE_KD 5.0
#define STEERING_KP 20.0
#define STEERING_KI 0.0
#define STEERING_KD 2.0

// Encoder configuration
#define ENCODER_TICKS_PER_REV 2400  // TODO: Set to actual value

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Cat position from vision system
 */
typedef struct {
    float x, y, confidence;     // Position and confidence
    uint64_t timestamp_ns;      // When received
    int detected;               // 1 if currently detected
} cat_position_t;

/**
 * @brief PID controller state
 */
typedef struct {
    float kp, ki, kd;           // Gains
    float integrator;           // Integral accumulator
    float prev_error;           // Previous error (for derivative)
    float dt;                   // Sample time
    float integrator_max;       // Anti-windup limit
} pid_controller_t;

/**
 * @brief Robot operating mode
 */
typedef enum {
    MODE_IDLE,                  // Motors off
    MODE_BALANCE,               // Balance only, no movement
    MODE_FOLLOW_CAT,            // Active cat following
    MODE_MANUAL                 // Xbox controller input
} robot_mode_t;

/**
 * @brief Complete robot state
 */
typedef struct {
    // IMU state
    float theta;                // Body pitch angle (rad)
    float theta_dot;            // Body pitch rate (rad/s)
    float phi;                  // Body roll angle (rad)
    float psi;                  // Body yaw angle (rad)
    
    // Encoder state
    float phi_left;             // Left wheel angle (rad)
    float phi_right;            // Right wheel angle (rad)
    
    // Control references
    float theta_ref;            // Desired body angle (for D1)
    float steering;             // Desired steering (-1 to 1)
    
    // Cat tracking
    cat_position_t cat;
    
    // Robot state
    robot_mode_t mode;
    int armed;                  // 0=disarmed, 1=armed
} robot_state_t;

// ============================================================================
// GLOBAL STATE (defined in robot.c)
// ============================================================================

extern robot_state_t state;
extern rc_mpu_data_t mpu_data;
extern pid_controller_t balance_pid;
extern pid_controller_t steering_pid;
extern debug_config_t g_debug_config;
extern telemetry_data_t g_telemetry_data;

// ============================================================================
// PID CONTROLLER FUNCTIONS (pid.c)
// ============================================================================

/**
 * @brief Initialize PID controller
 * 
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param dt Sample time (seconds)
 */
void pid_init(pid_controller_t* pid, float kp, float ki, float kd, float dt);

/**
 * @brief Update PID controller
 * 
 * @param pid Pointer to PID controller
 * @param setpoint Desired value
 * @param measurement Actual value
 * @return Controller output
 */
float pid_update(pid_controller_t* pid, float setpoint, float measurement);

/**
 * @brief Reset PID controller state
 * 
 * Clears integrator and error history. Call when arming/disarming.
 * 
 * @param pid Pointer to PID controller
 */
void pid_reset(pid_controller_t* pid);

/**
 * @brief Update PID gains at runtime
 * 
 * Allows tuning from iPhone app without restarting.
 * 
 * @param pid Pointer to PID controller
 * @param kp New proportional gain
 * @param ki New integral gain
 * @param kd New derivative gain
 */
void pid_set_gains(pid_controller_t* pid, float kp, float ki, float kd);

// ============================================================================
// CAT TRACKER FUNCTIONS (cat_tracker.c)
// ============================================================================

/**
 * @brief Initialize cat tracker UART communication
 * 
 * Opens UART connection to RPi5 running cat_track.
 * Starts background thread to receive cat position data.
 * 
 * @return 0 on success, -1 on error
 */
int cat_tracker_init(void);

/**
 * @brief Get current cat position
 * 
 * Retrieves the most recently received cat position.
 * Marks cat as not detected if data is stale (> CAT_TIMEOUT_MS).
 * 
 * @param pos Pointer to structure to fill with cat position
 * @return 1 if cat detected, 0 if not detected or stale
 */
int cat_tracker_get_position(cat_position_t* pos);

/**
 * @brief Cleanup cat tracker
 * 
 * Stops background thread and closes UART.
 */
void cat_tracker_cleanup(void);

// ============================================================================
// ROBOT CONTROL FUNCTIONS (robot.c)
// ============================================================================

/**
 * @brief Initialize robot control system
 * 
 * Initializes all hardware (IMU, motors, encoders, buttons)
 * and starts the control loop.
 * 
 * @return 0 on success, -1 on error
 */
int robot_init(void);

/**
 * @brief Main robot control loop
 * 
 * Blocks until exit signal received (Ctrl+C or button hold).
 */
void robot_run(void);

/**
 * @brief Cleanup robot control system
 * 
 * Stops motors, closes hardware, and cleans up resources.
 */
void robot_cleanup(void);

// ============================================================================
// IPC SERVER FUNCTIONS (ipc_server.c)
// ============================================================================

/**
 * @brief Initialize IPC server for iPhone communication
 * 
 * Creates Unix domain socket at /tmp/balance_bot.sock and
 * starts server thread to accept connections from Node.js.
 * 
 * @return 0 on success, -1 on error
 */
int ipc_server_init(void);

/**
 * @brief Broadcast telemetry to all connected clients
 * 
 * Sends current telemetry data (JSON format) to all connected
 * iPhone apps via Node.js server.
 * 
 * Call this periodically (e.g., 10 Hz) from main loop.
 */
void ipc_broadcast_telemetry(void);

/**
 * @brief Cleanup IPC server
 * 
 * Closes all connections and removes socket file.
 */
void ipc_server_cleanup(void);

// ============================================================================
// TELEMETRY FUNCTIONS (telemetry.c)
// ============================================================================

/**
 * @brief Initialize telemetry system
 * 
 * Clears telemetry data and sets up tracking state.
 * 
 * @return 0 on success, -1 on error
 */
int telemetry_init(void);

/**
 * @brief Update all telemetry data
 * 
 * Collects data from all sources (IMU, encoders, PIDs, etc.)
 * and populates g_telemetry_data structure.
 * 
 * Call this from main control loop before broadcasting.
 */
void telemetry_update(void);

/**
 * @brief Get human-readable telemetry configuration
 * 
 * @param buffer Output buffer
 * @param size Size of buffer
 */
void telemetry_get_config_description(char* buffer, size_t size);

/**
 * @brief Print telemetry summary to console
 * 
 * Useful for debugging without iPhone app.
 */
void telemetry_print_summary(void);

// ============================================================================
// PID CONFIG FILE FUNCTIONS (pid_config.c)
// ============================================================================

/**
 * @brief PID configuration file structure
 */
typedef struct {
    float balance_angle;        // Balance angle offset
    struct {
        float kp, ki, kd;
    } D1_balance;
    struct {
        float kp, ki, kd;
    } D2_drive;
    struct {
        float kp, ki, kd;
    } D3_steering;
} pid_config_file_t;

/**
 * @brief Load PID configuration from file
 * 
 * @param filename Path to config file
 * @param config Pointer to structure to fill
 * @return 0 on success, -1 on error
 */
int pid_config_load(const char* filename, pid_config_file_t* config);
int pid_config_load_or_default(const char* filename, pid_config_file_t* config);
void pid_config_print(const pid_config_file_t* config);

/**
 * @brief Save PID configuration to file
 * 
 * @param filename Path to config file
 * @param config Pointer to configuration
 * @return 0 on success, -1 on error
 */
int pid_config_save(const char* filename, const pid_config_file_t* config);

/**
 * @brief Apply loaded configuration to PIDs
 * 
 * Updates the global PID controllers with loaded config.
 * 
 * @param config Pointer to configuration
 */
void pid_config_apply(const pid_config_file_t* config);

// ============================================================================
// XBOX CONTROLLER FUNCTIONS (input_xbox.c)
// ============================================================================

/**
 * @brief Initialize Xbox controller input
 * 
 * @param device Device path (e.g., "/dev/input/js0")
 * @return 0 on success, -1 on error
 */
int xbox_init(const char* device);

/**
 * @brief Read Xbox controller state
 * 
 * Non-blocking read of joystick events.
 * Updates internal state for drive/turn/arm commands.
 * 
 * @return 0 on success, -1 on error
 */
int xbox_update(void);

/**
 * @brief Get Xbox controller drive command
 * 
 * @return Drive value (-1.0 to 1.0)
 */
float xbox_get_drive(void);

/**
 * @brief Get Xbox controller turn command
 * 
 * @return Turn value (-1.0 to 1.0)
 */
float xbox_get_turn(void);

/**
 * @brief Check if arm button pressed
 * 
 * @return 1 if pressed this frame, 0 otherwise
 */
int xbox_get_arm_button(void);

/**
 * @brief Cleanup Xbox controller
 */
void xbox_cleanup(void);

// ============================================================================
// SBUS INPUT FUNCTIONS (input_sbus.c)
// ============================================================================

/**
 * @brief Initialize SBUS input
 *
 * @param device  UART device path e.g. "/dev/ttyO1". NULL = use default.
 * @return 0 on success, -1 on error
 */
int sbus_init(const char *device);

/**
 * @brief Read and process pending SBUS bytes — call at 100 Hz
 *
 * @return 1=frame decoded, 0=no complete frame yet, -1=error
 */
int sbus_update(void);

/** @brief Cleanup SBUS — close UART */
void sbus_cleanup(void);

/** @brief Forward/back drive command, scaled by speed mode (-1 to +1) — CH2 Ele */
float sbus_get_drive(void);

/** @brief Yaw/turn command, scaled by speed mode (-1 to +1) — CH1 Ail */
float sbus_get_turn(void);

/** @brief Arm state from CH5 SA 3-pos switch: 0=low(disarmed) 1=mid 2=high(armed) */
int sbus_get_arm(void);

/** @brief Kill switch from CH6 SB 3-pos switch: 0=low(kill) 1=mid(kill) 2=high(run) */
int sbus_get_kill(void);

/** @brief Speed mode from CH10 SD 3-pos switch: 0=slow 1=normal 2=sport */
int sbus_get_speed_mode(void);

/** @brief True if receiver is in hardware failsafe (TX link lost) */
bool sbus_get_failsafe(void);

/** @brief True if UART is open and receiving frames */
bool sbus_is_connected(void);

/** @brief Aux analogue 1 from CH7 S1 pot/slider (-1.0 to +1.0) */
float sbus_get_aux1(void);

/** @brief Aux analogue 2 from CH8 S2 pot/slider (-1.0 to +1.0) */
float sbus_get_aux2(void);

/** @brief Aux switch C from CH9 SC 3-pos (0=low 1=mid 2=high) */
int sbus_get_sw_c(void);

/** @brief Aux switch E from CH11 SE 3-pos (0=low 1=mid 2=high) */
int sbus_get_sw_e(void);

/** @brief Aux switch F from CH12 SF 2-pos (true = high) */
bool sbus_get_sw_f(void);

/**
 * @brief Raw 11-bit value for any channel (0-indexed, range 172-1811)
 *
 * Use this to access stub channels CH6-CH16 without modifying input_sbus.c.
 * @param ch  Channel index 0-15
 */
uint16_t sbus_get_channel_raw(int ch);

/**
 * @brief Normalised float for any channel (0-indexed, range -1.0 to +1.0)
 * @param ch  Channel index 0-15
 */
float sbus_get_channel_float(int ch);

/**
 * @brief IMU axis mapping
 */
typedef enum {
    IMU_AXIS_X = 0,
    IMU_AXIS_Y = 1,
    IMU_AXIS_Z = 2
} imu_axis_t;

/**
 * @brief IMU orientation configuration
 * 
 * Defines how IMU axes map to robot axes
 */
typedef struct {
    imu_axis_t pitch_axis;    // Which IMU axis gives robot pitch
    imu_axis_t yaw_axis;      // Which IMU axis gives robot yaw
    imu_axis_t roll_axis;     // Which IMU axis gives robot roll
    
    float pitch_sign;         // +1 or -1
    float yaw_sign;           // +1 or -1
    float roll_sign;          // +1 or -1
} imu_orientation_t;

/**
 * @brief IMU calibration offsets
 */
typedef struct {
    float pitch_offset;       // Zero point for pitch (rad)
    float yaw_offset;         // Zero point for yaw (rad)
    float roll_offset;        // Zero point for roll (rad)
    
    float gyro_x_offset;      // Gyro X drift compensation (rad/s)
    float gyro_y_offset;      // Gyro Y drift compensation (rad/s)
    float gyro_z_offset;      // Gyro Z drift compensation (rad/s)
} imu_offsets_t;

/**
 * @brief Complete IMU configuration
 */
typedef struct {
    imu_orientation_t orientation;
    imu_offsets_t offsets;
    bool calibrated;
} imu_config_t;

/**
 * @brief Transformed IMU data in robot coordinates
 */
typedef struct {
    float pitch;              // Forward/back lean (rad)
    float yaw;                // Rotation (rad)
    float roll;               // Side lean (rad) - should be ~0
    
    float pitch_dot;          // Pitch rate (rad/s)
    float yaw_dot;            // Yaw rate (rad/s)
    float roll_dot;           // Roll rate (rad/s)
    
    float accel_x;            // Raw accel (m/s²)
    float accel_y;
    float accel_z;
} imu_transform_t;

// Global IMU configuration
extern imu_config_t g_imu_config;

// IMU configuration functions
imu_config_t imu_config_get_default(void);
int imu_config_load(imu_config_t* config);
int imu_config_save(const imu_config_t* config);
void imu_config_apply_transform(const rc_mpu_data_t* raw, imu_transform_t* transformed, const imu_config_t* config);
void imu_config_calibrate(const rc_mpu_data_t* raw, imu_config_t* config);
void imu_config_print(const imu_config_t* config);

// ============================================================================
// UTILITY MACROS
// ============================================================================

/**
 * Saturate float value to range (FIXED - dereference pointer)
 */
#define rc_saturate_float(val, min, max) \
    do { \
        if (*(val) < (min)) *(val) = (min); \
        else if (*(val) > (max)) *(val) = (max); \
    } while(0)

/**
 * Degrees to radians
 */
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif

/**
 * Radians to degrees
 */
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif

#endif // CAT_FOLLOWER_H
