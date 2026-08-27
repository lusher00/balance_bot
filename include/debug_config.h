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
typedef struct
{
    int32_t left_ticks; // Raw encoder ticks
    int32_t right_ticks;
    float left_rad; // Position in radians
    float right_rad;
    float left_vel; // Velocity in rad/s
    float right_vel;
} encoder_telemetry_t;

/**
 * @brief IMU telemetry data (full state)
 */
typedef struct
{
    // Attitude (orientation) - transformed via imu_config, used by PID
    float theta; // Pitch - forward/back lean (rad)
    float phi;   // Roll - side lean (rad)
    float psi;   // Yaw - rotation (rad)

    // Angular rates
    float theta_dot; // Pitch rate (rad/s)
    float phi_dot;   // Roll rate (rad/s)
    float psi_dot;   // Yaw rate (rad/s)

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
 *
 * Used by balance and steering ONLY. Both are genuine PID loops
 * running pid_update(). position hold is not — see drive_telemetry_t below.
 */
typedef struct
{
    bool enabled;      // Is this controller active?
    float setpoint;    // Desired value
    float measurement; // Actual value
    float error;       // setpoint - measurement
    float p_term;      // Proportional term
    float i_term;      // Integral term
    float d_term;      // Derivative term
    float output;      // Final controller output
    float kp, ki, kd;  // Current gains (for app sync on connect)
} pid_telemetry_t;

/**
 * @brief position hold drive (position-hold) controller state — NOT A PID
 *
 * position hold does not run pid_update() and has no gains. It is a zone-based
 * gain-scheduled position hold implemented inline in robot.c:
 *
 *   1. err = enc_pos_target - enc_pos                      (encoder ticks)
 *   2. correction = err / scale_X, where scale_X is chosen by |err|
 *      falling into zone A/B/C/D                           (degrees)
 *   3. correction is slew-rate-limited by max_angle_rate    (deg/tick)
 *   4. velocity damping is subtracted AFTER the rate limiter
 *   5. result is hard-clamped to +/- max_correction and written to theta_ref
 *
 * There is no proportional gain, no integrator, and no derivative. Do not
 * add p_term/i_term/d_term/kp/ki/kd fields here — the previous revision
 * reused pid_telemetry_t and the resulting CSV exports were misread as
 * PID traces. Field names below describe what is actually computed.
 */
typedef struct
{
    bool enabled;           // Is this controller active?
    int32_t enc_pos_target; // Target tick position being held
    int32_t enc_pos;        // Current tick position
    int32_t enc_error;      // enc_pos_target - enc_pos (+ve = behind target, must
                            // drive forward). Same sign as the err the zone logic
                            // divides, and as balance/steering's (setpoint - measurement).
    float enc_velocity;     // Tick velocity (ticks per 100 ms window)
    /* The RoboClaw's own speed reading, same units. Carried ONLY so the new
     * 100 Hz least-squares estimate can be compared against the 25 Hz staircase
     * it replaced, in the same log. Nothing controls off this. */
    float enc_velocity_raw;
    float enc_vel_lsq_mid;   /* candidate: mid window,  logged only */
    float enc_vel_lsq_long;  /* candidate: long window, logged only */
    float pos_correction;   // Zone-scheduled correction, pre-limit (deg)
    float vel_damp;         // Velocity damping applied (deg)
    float theta_ref_adj;    // Final rate-limited, clamped value written to theta_ref (deg)
    float active_scale;     // Zone divisor in effect this tick (0 = deadband/none)
    float max_correction;   // Clamp limit currently configured (deg)
} drive_telemetry_t;

/**
 * @brief Motor command telemetry
 */
typedef struct
{
    float left_duty;  // Left motor duty cycle (-1 to 1)
    float right_duty; // Right motor duty cycle (-1 to 1)
} motor_telemetry_t;

/**
 * @brief External UART input telemetry
 */
typedef struct
{
    bool valid;       // Is data fresh?
    float x;          // Lateral command (-1 to 1)
    float y;          // Forward command (-1 to 1)
    float confidence; // Source confidence (0 to 1)
} ext_input_telemetry_t;

/**
 * @brief System status telemetry
 */
typedef enum
{
    BATT_UNKNOWN = 0,
    BATT_OK = 1,
    BATT_WARNING = 2,
    BATT_CRITICAL = 3,
} batt_status_t;

typedef struct
{
    float battery_voltage; // Battery voltage (V) — onboard ADC (legacy)
    bool armed;            // Are motors armed?
    int mode;              // 0=balance, 1=ext_input, 2=manual
    float loop_hz;         // Actual control loop frequency
    uint32_t uptime_sec;   // Seconds since start
    float theta_offset;    // Balance point trim (deg)
    float batt_voltage;
    batt_status_t batt_status;
    float claw_voltage; // RoboClaw main battery voltage (V), 0 if unavailable
    float claw_temp;    // RoboClaw board temperature (°C), 0 if unavailable

    /* ── board health ─────────────────────────────────────────────────────
     * Sampled at 1 Hz from /proc and the thermal zone. On a single-core
     * 1 GHz board the control loop, the node bridge, sshd and this process
     * all share one CPU, so "is the robot fine but the box overloaded?" is a
     * question that comes up constantly -- and answering it has meant sshing
     * in and running top, at exactly the moment ssh is the thing that is
     * sluggish. These put the answer on the dashboard instead.
     *
     * ctxt_per_s earns its place specifically: the SBUS byte-at-a-time bug
     * showed 60%+ idle CPU while ssh crawled, and the context-switch rate was
     * the only number that showed it. */
    float cpu_pct;       // whole-board CPU busy, 0-100
    float bot_cpu_pct;   // this process's share, 0-100
    float load1;         // 1-minute load average
    uint32_t mem_avail_kb;
    uint32_t mem_total_kb;
    float sys_temp_c;    // SoC temperature
    uint32_t ctxt_per_s; // context switches/sec
    uint32_t procs_running;

    /* ── the other bot processes ──────────────────────────────────────────
     * Whether each is alive, and what share of the core it is taking.
     *
     * This exists because the node bridge was measured at 36% of a single
     * core and nothing on the dashboard said so -- the only way to find out
     * was to ssh in and run htop, at exactly the moment ssh was unusable
     * BECAUSE of that load. A service being up is not the same as it being
     * affordable, and the dashboard should show both. */
    bool node_alive;      // balance_bot_server (the websocket bridge)
    float node_cpu_pct;
    bool oled_alive;      // bbb_oled.py
    float oled_cpu_pct;
    bool batt_alive;      // batt_monitor
    float batt_cpu_pct;
} system_telemetry_t;

/**
 * @brief Complete telemetry packet
 *
 * This structure contains all possible telemetry data.
 * Only enabled fields are populated and transmitted.
 */
typedef struct
{
    uint64_t timestamp_us; // Microsecond timestamp

    encoder_telemetry_t encoders; // Encoder data
    imu_telemetry_t imu;          // IMU data

    pid_telemetry_t pitch;  // Balance PID state
    drive_telemetry_t position;  // Drive position-hold state (not a PID)
    pid_telemetry_t yaw; // Steering PID state

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
typedef struct
{
    bool encoders;       // Encoder ticks/position/velocity
    bool imu_attitude;   // Theta, phi, psi only (for 3D visualization)
    bool imu_full;       // All IMU data (accel, gyro) - HIGH BANDWIDTH
    bool pid_states;     // PID errors, terms, outputs
    bool motor_commands; // Motor duty cycles
    bool ext_input;      // External UART input data
    bool system_status;  // Battery, armed, mode - always recommended
} telemetry_enables_t;

/**
 * @brief Video overlay configuration
 *
 * Flags forwarded to an external coprocessor (e.g. RPi) to control
 * what is overlaid on any video stream it produces.
 */
typedef struct
{
    bool crosshair; // Draw crosshair / target marker
    bool stats;     // Show FPS / detection stats
} video_overlays_t;

/**
 * @brief Telemetry update rates (Hz)
 */
typedef struct
{
    int system_status;  // System status update rate (default: 1 Hz)
    int pid_states;     // PID state update rate (default: 10 Hz)
    int full_telemetry; // Full telemetry rate (default: 10 Hz, max: 100 Hz)
    /* RC (SBUS) packet rate. Deliberately much higher than pid_states: the
     * receiver delivers a frame every ~7 ms (~143 Hz), so at 10 Hz the RC
     * display saw 1 sample in 14 and a quick stick movement never appeared at
     * full deflection. The RC packet is ~270 bytes against ~1300 for telemetry,
     * so 50 Hz of RC costs less bandwidth than the old combined packet did at
     * 10 Hz. Values above the control loop rate (100 Hz) just send duplicates. */
    int rc;
} telemetry_rates_t;

/**
 * @brief Logging configuration
 */
typedef enum
{
    LOG_LEVEL_DEBUG, // Everything
    LOG_LEVEL_INFO,  // Normal operation
    LOG_LEVEL_WARN,  // Warnings only
    LOG_LEVEL_ERROR  // Errors only
} log_level_t;

typedef struct
{
    log_level_t level; // Current log level
    bool console;      // Print to console
    bool file;         // Write to log file
    bool timestamps;   // Include timestamps
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
typedef struct
{
    int sbus_tx;  /**< SBUS TX channel display   (0 = off, else ticks) */
    int pid;      /**< PID state display          (0 = off, else ticks) */
    int encoders; /**< Encoder display            (0 = off, else ticks) */
    int imu;      /**< IMU display                (0 = off, else ticks) */
    int motors;   /**< Motor duty display         (0 = off, else ticks) */
    int system;   /**< System status display      (0 = off, else ticks) */
} display_config_t;

/**
 * @brief Complete debug configuration
 *
 * This is the master configuration structure that controls all
 * debug features, telemetry, and logging.
 */
typedef struct
{
    telemetry_enables_t telemetry; // What telemetry to send
    video_overlays_t overlays;     // Video overlay settings
    bool debug_position;                 // Verbose position hold controller logging
    telemetry_rates_t rates;       // Update rates
    logging_config_t logging;      // Logging configuration
    display_config_t display;      // Console display blocks
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
#define LOG_DEBUG(fmt, ...)                                                                                              \
    do                                                                                                                   \
    {                                                                                                                    \
        if (g_debug_config.logging.level <= LOG_LEVEL_DEBUG)                                                             \
        {                                                                                                                \
            if (g_debug_config.logging.timestamps)                                                                       \
            {                                                                                                            \
                printf("[DEBUG][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot() / 1000000), ##__VA_ARGS__); \
            }                                                                                                            \
            else                                                                                                         \
            {                                                                                                            \
                printf("[DEBUG] " fmt "\n", ##__VA_ARGS__);                                                              \
            }                                                                                                            \
        }                                                                                                                \
    } while (0)

#define LOG_INFO(fmt, ...)                                                                                              \
    do                                                                                                                  \
    {                                                                                                                   \
        if (g_debug_config.logging.level <= LOG_LEVEL_INFO)                                                             \
        {                                                                                                               \
            if (g_debug_config.logging.timestamps)                                                                      \
            {                                                                                                           \
                printf("[INFO][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot() / 1000000), ##__VA_ARGS__); \
            }                                                                                                           \
            else                                                                                                        \
            {                                                                                                           \
                printf("[INFO] " fmt "\n", ##__VA_ARGS__);                                                              \
            }                                                                                                           \
        }                                                                                                               \
    } while (0)

#define LOG_WARN(fmt, ...)                                                                                                       \
    do                                                                                                                           \
    {                                                                                                                            \
        if (g_debug_config.logging.level <= LOG_LEVEL_WARN)                                                                      \
        {                                                                                                                        \
            if (g_debug_config.logging.timestamps)                                                                               \
            {                                                                                                                    \
                fprintf(stderr, "[WARN][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot() / 1000000), ##__VA_ARGS__); \
            }                                                                                                                    \
            else                                                                                                                 \
            {                                                                                                                    \
                fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__);                                                              \
            }                                                                                                                    \
        }                                                                                                                        \
    } while (0)

/* ── rate-limited logging for hot paths ──────────────────────────────────────
 *
 * An unthrottled LOG_WARN in a path that runs at loop rate (100 Hz) or SBUS
 * frame rate (~143 Hz) is not a log line. It is a several-hundred-hertz writer
 * to stderr, which systemd hands to journald, which -- if the journal is
 * persistent -- writes it to the SD card. That is enough to make the whole
 * board unresponsive and, sustained, to fail a write and trip
 * ext4 errors=remount-ro.
 *
 * The failure modes these logs report are exactly the ones that repeat: a
 * RoboClaw that times out once times out again 10 ms later, and a receiver in
 * failsafe is in failsafe for every frame until the link returns. So the log
 * volume is highest precisely when the board can least afford it.
 *
 * Logs at most once per `ms`, and reports how many it swallowed, so a storm is
 * still visible as a storm rather than silently thinned.
 *
 * Each call site gets its own counters (function-static inside the macro), so
 * two different throttled messages do not suppress each other.
 *
 * The "primed" flag is not decoration. This used to test `_lwe_last == 0` to
 * mean "has never fired", which is also a legitimate value of the clock: if
 * rc_nanos_since_boot() ever returns 0 -- on error, or on a first call inside
 * the first millisecond -- _lwe_last stays 0 and that branch is taken every
 * single time. The signature is distinctive: every occurrence logged, and
 * _lwe_skipped never incremented, so "[+N more suppressed]" never appears at
 * all. That is exactly what 577 MB of /tmp/balance_bot.log showed on
 * 2026-08-24 -- 3.45 M lines across 154 boots, zero suppression notices.
 * Never overload a sentinel onto a value the source can legitimately produce.
 */
#define LOG_WARN_EVERY(ms, fmt, ...)                                            \
    do                                                                          \
    {                                                                           \
        static uint64_t _lwe_last = 0;                                          \
        static bool _lwe_primed = false;                                        \
        static unsigned long _lwe_skipped = 0;                                  \
        uint64_t _lwe_now = rc_nanos_since_boot() / 1000000ULL;                 \
        if (!_lwe_primed || (_lwe_now - _lwe_last) >= (uint64_t)(ms))           \
        {                                                                       \
            if (_lwe_skipped)                                                   \
                LOG_WARN(fmt " [+%lu more suppressed]", ##__VA_ARGS__,          \
                         _lwe_skipped);                                         \
            else                                                                \
                LOG_WARN(fmt, ##__VA_ARGS__);                                   \
            _lwe_last = _lwe_now;                                               \
            _lwe_primed = true;                                                 \
            _lwe_skipped = 0;                                                   \
        }                                                                       \
        else                                                                    \
        {                                                                       \
            _lwe_skipped++;                                                     \
        }                                                                       \
    } while (0)

#define LOG_ERROR(fmt, ...)                                                                                                       \
    do                                                                                                                            \
    {                                                                                                                             \
        if (g_debug_config.logging.level <= LOG_LEVEL_ERROR)                                                                      \
        {                                                                                                                         \
            if (g_debug_config.logging.timestamps)                                                                                \
            {                                                                                                                     \
                fprintf(stderr, "[ERROR][%llu] " fmt "\n", (unsigned long long)(rc_nanos_since_boot() / 1000000), ##__VA_ARGS__); \
            }                                                                                                                     \
            else                                                                                                                  \
            {                                                                                                                     \
                fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__);                                                              \
            }                                                                                                                     \
        }                                                                                                                         \
    } while (0)

// ============================================================================
// DEFAULT CONFIGURATIONS
// ============================================================================

/**
 * @brief Get default debug configuration
 *
 * Returns a sensible default configuration suitable for most use cases.
 * Enables basic telemetry without overwhelming the network.
 */
static inline debug_config_t get_default_debug_config(void)
{
    debug_config_t config = {
        .telemetry = {
            /* Off. The Control tab's encoder card is gone -- nothing consumed
             * this. Position telemetry already carries enc_pos/enc_error, which
             * is what the graphs and the tune analysis actually use. */
            .encoders = false,
            .imu_attitude = true, // For 3D visualization
            .imu_full = false,    // Disable high-bandwidth data
            .pid_states = true,
            .motor_commands = false,
            .ext_input = true,
            .system_status = true},
        .overlays = {.crosshair = true, .stats = false},
        .rates = {
            .system_status = 1,   // 1 Hz
            /* 25 Hz, was 10. The control loop runs at 100 Hz, so 10 Hz sampled a
             * moving signal 10 times a second: with the transmitter off nothing
             * moves and the traces look smooth, but drive the bot and the graph
             * turns into angular steps. That is aliasing, not packet loss --
             * measurement showed telemetry arriving exactly on time at 10 Hz
             * while the graphs looked broken.
             *
             * 25 divides the 10 ms loop tick exactly (40 ms), so spacing is
             * perfectly even -- which matters for anything plotted -- and it
             * lines up with POS_VEL_PERIOD_MS, so the encoder velocity in each
             * packet is fresh rather than repeated. Costs ~25 KB/s. */
            /* 20 Hz, was 25, was originally 10.
             *
             * 25 telemetry + 50 rc = 75 messages/s through the node bridge, and
             * htop on the bot measured node at 36% of the single core with a
             * load average of 1.46 -- the run queue never emptying. That is
             * what made ssh crawl and the board unresponsive, and it is
             * transmitter-independent because the RC stream is emitted whether
             * or not a transmitter is on. Which is precisely why turning the TX
             * off never actually helped.
             *
             * 20 Hz still divides the 10 ms loop tick exactly (50 ms), so
             * spacing stays even, and still gives twice the graph sample
             * density of the original 10 Hz. Change at runtime with the
             * set_rates IPC command -- no rebuild -- and watch the HUD. */
            .pid_states = 20,     // 20 Hz
            .full_telemetry = 20, // 20 Hz
            /* 20 Hz, was 50. See pid_states above: 50 Hz of RC on top of the
             * telemetry stream is most of what saturated the bridge. 20 Hz is
             * still double the fidelity RC had before it was split out, and it
             * divides the tick exactly (50 ms). Raise it with set_rates once
             * the board is proven to have headroom. */
            .rc = 20              // 20 Hz — see telemetry_rates_t.rc
        },
        .logging = {.level = LOG_LEVEL_INFO, .console = true, .file = false, .timestamps = true},
        .display = {
            .sbus_tx = 0,  /* off by default — enable with -d sbus    */
            .pid = 0,      /* off by default — enable with -d pid     */
            .encoders = 0, /* off by default — enable with -d enc     */
            .imu = 0,      /* off by default — enable with -d imu     */
            .motors = 0,   /* off by default — enable with -d mot     */
            .system = 100, /* system status on by default  (~1 Hz)    */
        }};
    return config;
}

#endif // DEBUG_CONFIG_H
