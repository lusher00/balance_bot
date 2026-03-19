/**
 * @file balance_bot.h
 * @brief Main header for balance_bot
 *
 * Primary header — includes all module interfaces for the
 * balance_bot self-balancing robot with iPhone app integration.
 *
 * Architecture:
 * - D1 (balance):  Angle controller — keeps robot upright
 * - D2 (drive):    Position controller — drives via lean angle (optional)
 * - D3 (steering): Yaw controller — turns left/right
 * - uart_input:    Generic packet-based UART input (external coprocessor, etc.)
 * - roboclaw:      Packet-serial motor driver over a dedicated UART
 * - ipc_server:    Unix-socket bridge to Node.js / iPhone app
 * - telemetry:     Real-time state broadcast to iPhone
 */

#ifndef BALANCE_BOT_H
#define BALANCE_BOT_H

#include <robotcontrol.h>
#include <stdbool.h>
#include <stdint.h>
#include "debug_config.h"
#include "motor_hal.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define SAMPLE_RATE_HZ      100
#define DT                  0.01f

// Control limits
#define MAX_THETA_REF       17.0f   // Max lean angle command (deg)
#define MAX_STEERING        1.0f    // Max steering command (normalized)

// PID default gains (tunable via iPhone app)
#define BALANCE_KP   0.050f
#define BALANCE_KI   0.015f
#define BALANCE_KD   0.005f
#define STEERING_KP  0.050f
#define STEERING_KI  0.015f
#define STEERING_KD  0.005f
#define DRIVE_KP     0.050f
#define DRIVE_KI     0.015f
#define DRIVE_KD     0.005f

#define DRIVE_PHI_DEADZONE 2.0f

// Encoder configuration
#define ENCODER_TICKS_PER_REV 2400  // TODO: set to actual value

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
typedef struct {
    float    x;             // Lateral / steering command  (-1 to +1)
    float    y;             // Forward / drive command     (-1 to +1)
    float    confidence;    // Source confidence           ( 0 to  1)
    uint64_t timestamp_ns;  // rc_nanos_since_boot() at receipt
    int      valid;         // 1 = fresh, 0 = stale / no signal
} input_packet_t;

/**
 * @brief PID controller state
 */
typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_error;
    float dt;
    float integrator_max;
} pid_controller_t;

/**
 * @brief Robot operating mode
 */
typedef enum {
    MODE_IDLE,          // Motors off
    MODE_BALANCE,       // Balance only, no movement
    MODE_EXT_INPUT,     // External UART packet drives the bot
    MODE_MANUAL         // Xbox / SBUS controller
} robot_mode_t;

/**
 * @brief Complete robot state
 */
typedef struct {
    // IMU
    float theta;        // Body pitch angle  (deg)
    float theta_dot;    // Body pitch rate   (deg/s)
    float phi;          // Body roll angle   (deg)
    float psi;          // Body yaw angle    (deg)

    // Encoders
    int32_t enc_left;       // Raw encoder ticks (always updated)
    int32_t enc_right;
    float phi_left;         // Left wheel angle  (deg)
    float phi_right;        // Right wheel angle (deg)

    // D2 position controller
    float pos;              // Global wheel position (deg): avg wheel angle + theta
    float pos_setpoint;     // D2 position setpoint (deg)

    // Control references
    float theta_ref;    // Desired body angle  (deg)
    float theta_offset; // Balance point trim  (deg) — tunable from iPhone
    float steering;     // Desired steering    (-1 to +1)

    // External UART input (used only in MODE_EXT_INPUT)
    input_packet_t ext_input;

    robot_mode_t mode;
    int armed;          // 0 = disarmed, 1 = armed
} robot_state_t;

// ============================================================================
// GLOBAL STATE (defined in robot.c)
// ============================================================================

extern robot_state_t    state;
extern rc_mpu_data_t    mpu_data;
extern pid_controller_t balance_pid;
extern pid_controller_t steering_pid;
extern pid_controller_t drive_pid;
extern debug_config_t   g_debug_config;
extern telemetry_data_t g_telemetry_data;

// ============================================================================
// PID (pid.c)
// ============================================================================

void  pid_init    (pid_controller_t *pid, float kp, float ki, float kd, float dt);
float pid_update  (pid_controller_t *pid, float setpoint, float measurement);
void  pid_reset   (pid_controller_t *pid);
void  pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

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
int uart_input_init   (const char *device, int baud, int timeout_ms);
int uart_input_get    (input_packet_t *pkt);   // 1=valid, 0=stale
void uart_input_cleanup(void);

// ============================================================================
// ROBOT CONTROL (robot.c)
// ============================================================================

int  robot_init   (void);
void robot_run    (void);
void robot_cleanup(void);

// ============================================================================
// IPC SERVER (ipc_server.c)
// ============================================================================

int  ipc_server_init       (void);
void ipc_broadcast_telemetry(void);
void ipc_server_cleanup    (void);

// ============================================================================
// TELEMETRY (telemetry.c)
// ============================================================================

int  telemetry_init                  (void);
void telemetry_update                (void);
void telemetry_get_config_description(char *buf, size_t size);
void telemetry_print_summary         (void);

// ============================================================================
// PID CONFIG FILE (pid_config.c)
// ============================================================================

typedef struct {
    float balance_angle;
    struct { float kp, ki, kd; } D1_balance;
    struct { float kp, ki, kd; } D2_drive;
    struct { float kp, ki, kd; } D3_steering;
} pid_config_file_t;

int  pid_config_load          (const char *filename, pid_config_file_t *config);
int  pid_config_load_or_default(const char *filename, pid_config_file_t *config);
void pid_config_print         (const pid_config_file_t *config);
int  pid_config_save          (const char *filename, const pid_config_file_t *config);
void pid_config_apply         (const pid_config_file_t *config);
void pid_config_get_current   (pid_config_file_t *config);

// ============================================================================
// XBOX CONTROLLER (input_xbox.c)
// ============================================================================

int   xbox_init         (const char *device);
int   xbox_update       (void);
float xbox_get_drive    (void);
float xbox_get_turn     (void);
int   xbox_get_arm_button(void);
void  xbox_cleanup      (void);

// ============================================================================
// SBUS INPUT (input_sbus.c)
// ============================================================================

int      sbus_init             (const char *device);
int      sbus_update           (void);
void     sbus_cleanup          (void);
float    sbus_get_drive        (void);
float    sbus_get_turn         (void);
int      sbus_get_arm          (void);
int      sbus_get_kill         (void);
int      sbus_get_speed_mode   (void);
bool     sbus_get_failsafe     (void);
bool     sbus_is_connected     (void);
float    sbus_get_aux1         (void);
float    sbus_get_aux2         (void);
int      sbus_get_sw_c         (void);
int      sbus_get_sw_e         (void);
bool     sbus_get_sw_f         (void);
uint16_t sbus_get_channel_raw  (int ch);
float    sbus_get_channel_float(int ch);

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

typedef struct {
    float pitch_offset;      // angle when upright (degrees)
    float yaw_offset;        // yaw at heading zero (degrees)
    float pitch_dot_offset;  // gyro Y bias (deg/s) — corrects constant drift
} imu_offsets_t;

typedef struct {
    float pitch, yaw, roll;
    float pitch_dot, yaw_dot, roll_dot;
    float accel_x, accel_y, accel_z;
} imu_transform_t;

extern imu_offsets_t g_imu_offsets;

int  imu_offsets_load  (imu_offsets_t *offsets);
int  imu_offsets_save  (const imu_offsets_t *offsets);
void imu_offsets_calibrate(const rc_mpu_data_t *raw, imu_offsets_t *offsets);
void imu_apply_transform  (const rc_mpu_data_t *raw, imu_transform_t *out,
                            const imu_offsets_t *offsets);

// ============================================================================
// UTILITY MACROS
// ============================================================================

#define rc_saturate_float(val, mn, mx) \
    do { if (*(val) < (mn)) *(val) = (mn); \
         else if (*(val) > (mx)) *(val) = (mx); } while(0)

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif

#endif /* BALANCE_BOT_H */
