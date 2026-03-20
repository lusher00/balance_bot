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

// D2 position controller — encoder-tick-based hold/drive
// These defaults initialise g_pos_config in robot.c.
// Use set_pos_config IPC command or the iPhone app to tune at runtime.
#define POS_ZONE_A_DEFAULT          8000
#define POS_ZONE_B_DEFAULT          4000
#define POS_ZONE_C_DEFAULT          1000
#define POS_SCALE_A_DEFAULT         600.0f
#define POS_SCALE_B_DEFAULT         800.0f
#define POS_SCALE_C_DEFAULT         1000.0f
#define POS_SCALE_D_DEFAULT         500.0f
#define POS_VEL_SCALE_STOP_DEFAULT  60.0f
#define POS_VEL_SCALE_MOVE_DEFAULT  70.0f
#define POS_VEL_SCALE_TURNING_DEFAULT 70.0f  // Turning authority reduction at speed
#define POS_STOPPED_VEL_DEFAULT     40
#define POS_MAX_CORRECTION_DEFAULT  10.0f
#define POS_MAX_ANGLE_RATE_DEFAULT  5.0f     // deg/tick — Balanduino 1°@500Hz ≈ 5°@100Hz
#define POS_BACK_TO_SPOT_DEFAULT    1        // Full zone hold by default
#define POS_VEL_PERIOD_MS           100

/**
 * @brief Runtime-tunable parameters for the D2 position (hold/drive) controller.
 *
 * All fields are readable and writable at runtime via the IPC set_pos_config
 * command and the iPhone app.  Initialised from the _DEFAULT macros above.
 *
 * Zone thresholds (ticks): A > B > C, with D being the tightest deadband
 * (error inside zone C).  scale_* divides the raw tick error to produce a
 * lean-angle bias in degrees.  vel_scale_* divides the 100 ms tick velocity
 * for damping / back-EMF compensation.
 */
typedef struct {
    int32_t zone_a;         // Outer zone threshold (ticks)
    int32_t zone_b;
    int32_t zone_c;
    float   scale_a;        // Tick-error → lean-angle divisor, zone A
    float   scale_b;
    float   scale_c;
    float   scale_d;        // Inside zone C (tightest hold)
    float   vel_scale_stop; // Velocity damp divisor when holding
    float   vel_scale_move; // Back-EMF comp divisor when driving
    float   vel_scale_turning; // Reduces turning authority at speed (Balanduino-style)
    int32_t stopped_vel;    // Ticks/100ms threshold for "stopped" detection
    float   max_correction; // Maximum lean-angle correction D2 may inject (deg)
    float   max_angle_rate; // Max correction change per main loop tick (deg/tick)
                            // Rate-limits D2 output to prevent slamming theta_ref.
                            // Balanduino uses 1°/loop at 500Hz ≈ 5°/loop at 100Hz.
    int     back_to_spot;   // 1 = full zone-based hold (A/B/C/D);
                            // 0 = only correct inside zone_c (loose hold, Balanduino mode)
} pos_config_t;

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

    // D2 position controller (encoder-tick based)
    int32_t enc_pos;            // Sum of left+right encoder ticks (position)
    int32_t enc_pos_target;     // Target tick position (held when stick is centered)
    int32_t enc_velocity;       // Tick velocity (ticks per 100 ms window)
    int32_t enc_velocity_raw;   // Raw delta before the stopped-check
    int     enc_vel_reset;      // Set to 1 by ipc_server after zero_encoders; cleared by robot.c

    // Legacy degree-based position (kept for telemetry)
    float pos;              // avg wheel angle (deg)
    float pos_setpoint;     // D2 setpoint (deg) — unused when D2_drive enabled

    // Control references
    float theta_ref;    // Desired body angle  (deg)
    float theta_offset; // Balance point trim  (deg) — tunable from iPhone
    float steering;     // Desired steering    (-1 to +1)

    // D3 steering latch — when the drive stick returns to centre, D3 holds
    // the phi_diff at that moment rather than fighting back to zero.
    float steering_latch;   // phi_diff value latched at stick-centre transition
    int   steering_latched; // 1 = latch is active (stick centred), 0 = driving

    // External UART input (used only in MODE_EXT_INPUT)
    input_packet_t ext_input;

    robot_mode_t mode;
    int trying;
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
extern pos_config_t     g_pos_config;

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
// POSITION CONTROLLER CONFIG (pid_config.c)
// ============================================================================

/**
 * @brief Apply a pos_config_t to the global g_pos_config.
 * Called from IPC set_pos_config handler and on startup.
 */
void pos_config_apply  (const pos_config_t *cfg);

/**
 * @brief Populate *cfg from the current g_pos_config values.
 * Used by save_pid to persist position params alongside PID gains.
 */
void pos_config_get_current(pos_config_t *cfg);

/**
 * @brief Save pos_config to file (appended section in pidconfig.txt).
 */
int  pos_config_save   (const char *filename, const pos_config_t *cfg);

/**
 * @brief Load pos_config from file, or fill defaults if section absent.
 */
int  pos_config_load_or_default(const char *filename, pos_config_t *cfg);

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
