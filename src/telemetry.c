// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
//
// This file is part of balance_bot, licensed under the PolyForm
// Noncommercial License 1.0.0. You may use, study, modify, and share
// it for any noncommercial purpose. Commercial use requires a separate
// license from the author -- contact ryan.lush@gmail.com.
// Full license text: see the LICENSE file in the project root, or
// https://polyformproject.org/licenses/noncommercial/1.0.0/

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
#include "rc_compat.h"
#include "motor_hal.h"
#include <math.h>
#include <unistd.h>   /* sysconf(_SC_CLK_TCK) for CPU accounting */
#include <dirent.h>   /* scanning /proc for the other bot processes */
#include <sys/stat.h> /* stat() on /run/batt_status.json */
#include <time.h>

// Global telemetry data (shared with IPC server)
telemetry_data_t g_telemetry_data = {0};

// External references to robot state
extern robot_state_t state;
extern rc_mpu_data_t mpu_data;
extern pid_controller_t pitch_pid, yaw_pid;

// Encoder tracking for velocity calculation
static int32_t prev_left_ticks = 0;
static int32_t prev_right_ticks = 0;
static uint64_t prev_timestamp_us = 0;

/**
 * @brief Update encoder telemetry
 *
 * Reads encoder values and calculates position and velocity.
 */
static void update_encoder_telemetry(void)
{
    // Encoder positions are always tracked by robot.c — just copy from state.
    uint64_t now_us = rc_nanos_since_boot() / 1000;

    g_telemetry_data.encoders.left_ticks = state.enc_left;
    g_telemetry_data.encoders.right_ticks = state.enc_right;
    g_telemetry_data.encoders.left_rad = state.phi_left;
    g_telemetry_data.encoders.right_rad = state.phi_right;

    if (prev_timestamp_us > 0)
    {
        float dt = (now_us - prev_timestamp_us) / 1000000.0f;
        if (dt > 0.0001f)
        {
            int32_t left_delta = state.enc_left - prev_left_ticks;
            int32_t right_delta = state.enc_right - prev_right_ticks;
            g_telemetry_data.encoders.left_vel =
                (left_delta * (360.0f / ENCODER_TICKS_PER_REV)) / dt;
            g_telemetry_data.encoders.right_vel =
                (right_delta * (360.0f / ENCODER_TICKS_PER_REV)) / dt;
        }
    }

    prev_left_ticks = state.enc_left;
    prev_right_ticks = state.enc_right;
    prev_timestamp_us = now_us;
}

/**
 * @brief Update IMU telemetry
 *
 * Uses the same imu_config_apply_transform() path that the PID controller
 * uses, so the angles the app sees match exactly what the robot is acting on.
 * Raw DMP values are NOT used here — that was the previous bug.
 */
static void update_imu_telemetry(void)
{
    if (!g_debug_config.telemetry.imu_attitude)
        return;

    // DMP quaternion (accel/gyro only, no magnetometer required)
    // rc_mpu_data_t.dmp_quat[4] is ordered [W, X, Y, Z]
    g_telemetry_data.imu.qw = (float)mpu_data.dmp_quat[0];
    g_telemetry_data.imu.qx = (float)mpu_data.dmp_quat[1];
    g_telemetry_data.imu.qy = (float)mpu_data.dmp_quat[2];
    g_telemetry_data.imu.qz = (float)mpu_data.dmp_quat[3];
    imu_transform_t t;
    imu_apply_transform(&mpu_data, &t, &g_imu_offsets);

    g_telemetry_data.imu.theta = t.pitch - state.theta_offset;
    g_telemetry_data.imu.phi = t.roll;
    g_telemetry_data.imu.psi = t.yaw;
    g_telemetry_data.imu.theta_dot = t.pitch_dot;
    g_telemetry_data.imu.phi_dot = t.roll_dot;
    g_telemetry_data.imu.psi_dot = t.yaw_dot;

    // Full IMU data (if enabled)
    if (g_debug_config.telemetry.imu_full)
    {
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
static void update_pid_telemetry(void)
{
    if (!g_debug_config.telemetry.pid_states)
        return;

    // Balance controller
    g_telemetry_data.pitch.enabled = g_controllers.pitch;
    g_telemetry_data.pitch.setpoint = state.theta_ref + state.theta_offset;
    g_telemetry_data.pitch.measurement = state.theta;
    g_telemetry_data.pitch.error = pitch_pid.prev_error;
    // Read the terms recorded by pid_update() rather than recomputing them.
    // d_term cannot be reconstructed here (prev_error has already advanced),
    // and output must be the real controller output — a p+i reconstruction
    // silently hides the derivative contribution from every log and graph.
    g_telemetry_data.pitch.p_term = pitch_pid.last_p_term;
    g_telemetry_data.pitch.i_term = pitch_pid.last_i_term;
    g_telemetry_data.pitch.d_term = pitch_pid.last_d_term;
    g_telemetry_data.pitch.output = pitch_pid.last_output;
    g_telemetry_data.pitch.kp = pitch_pid.kp;
    g_telemetry_data.pitch.ki = pitch_pid.ki;
    g_telemetry_data.pitch.kd = pitch_pid.kd;

    // Position hold (position-hold) controller — NOT a PID. See drive_telemetry_t.
    // Every field is named for the quantity it actually carries. Do not
    // reintroduce setpoint/measurement/p_term/i_term/d_term aliases here.
    g_telemetry_data.position.enabled = g_controllers.position;
    g_telemetry_data.position.enc_pos_target = state.enc_pos_target;
    g_telemetry_data.position.enc_pos = state.enc_pos;
    // Sign matches robot.c: err = enc_pos_target - enc_pos, which is the error
    // the zone logic actually divides by active_scale. It also matches balance/steering,
    // which log (setpoint - measurement). Logging enc_pos - enc_pos_target here
    // made every trace show error and pos_correction moving in opposite
    // directions, which is exactly as misleading as the old p_term aliases.
    g_telemetry_data.position.enc_error = state.enc_pos_target - state.enc_pos;
    g_telemetry_data.position.enc_velocity = state.enc_velocity;
    g_telemetry_data.position.enc_velocity_raw = state.enc_velocity_raw;
    g_telemetry_data.position.enc_vel_lsq_mid  = state.enc_vel_lsq_mid;
    g_telemetry_data.position.enc_vel_lsq_long = state.enc_vel_lsq_long;
    g_telemetry_data.position.pos_correction = state.pos_correction;
    g_telemetry_data.position.vel_damp = state.pos_vel_damp;
    g_telemetry_data.position.theta_ref_adj = state.pos_output;
    g_telemetry_data.position.active_scale = state.pos_scale;
    g_telemetry_data.position.max_correction = g_pos_config.max_correction;

    // Steering controller
    g_telemetry_data.yaw.enabled = g_controllers.yaw;
    g_telemetry_data.yaw.setpoint = state.yaw;
    /* Same sign as the control loop. This was (phi_left - phi_right)/2 while
     * robot.c closes on (phi_right - phi_left)/2, so every log and every
     * dashboard reading had yaw_measurement NEGATED relative to yaw_setpoint
     * and yaw_error -- yaw_error did not equal setpoint minus measurement, and
     * reading the columns at face value gave the wrong answer about which way
     * the robot was turning. */
    g_telemetry_data.yaw.measurement = (state.phi_right - state.phi_left) / 2.0f;  // deg diff
    g_telemetry_data.yaw.error = yaw_pid.prev_error;
    g_telemetry_data.yaw.p_term = yaw_pid.last_p_term;
    g_telemetry_data.yaw.i_term = yaw_pid.last_i_term;
    g_telemetry_data.yaw.d_term = yaw_pid.last_d_term;
    g_telemetry_data.yaw.output = yaw_pid.last_output;
    g_telemetry_data.yaw.kp = yaw_pid.kp;
    g_telemetry_data.yaw.ki = yaw_pid.ki;
    g_telemetry_data.yaw.kd = yaw_pid.kd;
} /**
   * @brief Update motor command telemetry
   */
static void update_motor_telemetry(void)
{
    if (!g_debug_config.telemetry.motor_commands)
        return;

    // Motor duty cycles are set in the control loop
    // We can read them back if needed, or track them
    // For now, leaving as 0 - implement if motor readback is available
    // Motor duty is written by the control loop itself (robot.c, from
    // last_left_duty / last_right_duty) immediately after telemetry_update()
    // returns. Do NOT zero it here — that used to blank the values every tick
    // and only worked by accident of ordering.
}

/**
 * @brief Update external UART input telemetry
 */
static void update_ext_input_telemetry(void)
{
    if (!g_debug_config.telemetry.ext_input)
        return;

    // Copy latest external input from robot state
    g_telemetry_data.ext_input.valid = state.ext_input.valid;
    g_telemetry_data.ext_input.x = state.ext_input.x;
    g_telemetry_data.ext_input.y = state.ext_input.y;
    g_telemetry_data.ext_input.confidence = state.ext_input.confidence;
}

/**
 * @brief CPU share of another process, found by command line, sampled at 1 Hz.
 *
 * Matching on /proc/<pid>/cmdline rather than comm, because the interesting
 * processes are interpreters: comm is "node" or "python3" for several unrelated
 * things, and the argument is what identifies them.
 *
 * The PID is cached and revalidated with a single read; a full /proc scan only
 * happens when the cached one stops matching. Scanning all of /proc three times
 * a second to re-find three processes that restart maybe twice a day was
 * measured elsewhere in this project at ~60x more work than needed.
 */
typedef struct
{
    const char *needle;      /* substring to look for in cmdline */
    pid_t pid;               /* cached */
    unsigned long long prev_jiffies;
    bool alive;
    float cpu_pct;
} proc_watch_t;

static bool proc_cmdline_matches(pid_t pid, const char *needle)
{
    char path[64], buf[512];
    snprintf(path, sizeof(path), "/proc/%d/cmdline", (int)pid);
    FILE *f = fopen(path, "r");
    if (!f)
        return false;
    /* cmdline is NUL-separated; turn it into one searchable string. */
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0)
        return false;
    for (size_t i = 0; i + 1 < n; i++)
        if (buf[i] == '\0')
            buf[i] = ' ';
    buf[n] = '\0';
    return strstr(buf, needle) != NULL;
}

static unsigned long long proc_jiffies(pid_t pid)
{
    char path[64], buf[1024];
    snprintf(path, sizeof(path), "/proc/%d/stat", (int)pid);
    FILE *f = fopen(path, "r");
    if (!f)
        return 0;
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0)
        return 0;
    buf[n] = '\0';
    char *p = strrchr(buf, ')');
    if (!p)
        return 0;
    unsigned long long ut = 0, st = 0;
    int field = 2;
    for (p++; *p && field < 14; p++)
        if (*p == ' ')
            field++;
    if (sscanf(p, "%llu %llu", &ut, &st) != 2)
        return 0;
    return ut + st;
}

static void proc_watch_sample(proc_watch_t *w, double dt, long hz)
{
    if (w->pid <= 0 || !proc_cmdline_matches(w->pid, w->needle))
    {
        w->pid = 0;
        DIR *d = opendir("/proc");
        if (d)
        {
            struct dirent *e;
            while ((e = readdir(d)))
            {
                if (e->d_name[0] < '0' || e->d_name[0] > '9')
                    continue;
                pid_t cand = (pid_t)atoi(e->d_name);
                if (proc_cmdline_matches(cand, w->needle))
                {
                    w->pid = cand;
                    w->prev_jiffies = 0;   /* restart the delta */
                    break;
                }
            }
            closedir(d);
        }
    }

    if (w->pid <= 0)
    {
        w->alive = false;
        w->cpu_pct = 0.0f;
        w->prev_jiffies = 0;
        return;
    }

    w->alive = true;
    unsigned long long j = proc_jiffies(w->pid);
    if (w->prev_jiffies && dt > 0.0 && j >= w->prev_jiffies && hz > 0)
        w->cpu_pct = 100.0f * (float)(j - w->prev_jiffies) / (float)hz / (float)dt;
    w->prev_jiffies = j;
}

/**
 * @brief Sample board health from /proc, at 1 Hz.
 *
 * Everything here is a delta against the previous sample, because the absolute
 * counters in /proc are meaningless on their own -- /proc/stat's jiffies are
 * cumulative since boot, so a single read tells you nothing about right now.
 *
 * Cost: five small reads from procfs once a second, on a control loop that
 * ticks a hundred times a second. Deliberately NOT sampled per tick: these are
 * board-scale numbers that do not move meaningfully at 100 Hz, and reading them
 * that often would make this the very load it is meant to detect.
 */
static void update_board_health(void)
{
    static uint64_t last_us = 0;
    static unsigned long long prev_busy = 0, prev_total = 0;
    static unsigned long long prev_ctxt = 0;
    static unsigned long long prev_self = 0;
    static long hz = 0;

    uint64_t now_us = rc_nanos_since_boot() / 1000;
    if (last_us != 0 && (now_us - last_us) < 1000000ULL)
        return;
    double dt = last_us ? (now_us - last_us) / 1000000.0 : 0.0;
    last_us = now_us;

    if (hz == 0)
        hz = sysconf(_SC_CLK_TCK);

    FILE *f;

    /* /proc/stat: whole-board CPU, context switches, runnable processes */
    if ((f = fopen("/proc/stat", "r")))
    {
        char line[512];
        while (fgets(line, sizeof(line), f))
        {
            if (strncmp(line, "cpu ", 4) == 0)
            {
                unsigned long long v[10] = {0};
                int n = sscanf(line + 4,
                               "%llu %llu %llu %llu %llu %llu %llu %llu %llu %llu",
                               &v[0], &v[1], &v[2], &v[3], &v[4],
                               &v[5], &v[6], &v[7], &v[8], &v[9]);
                unsigned long long total = 0;
                for (int i = 0; i < n; i++)
                    total += v[i];
                /* idle = idle + iowait. iowait counts as not-busy on purpose:
                 * a board blocked on the SD card is a different fault from one
                 * that is compute-bound, and lumping them together hides the
                 * fsync-storm signature entirely. */
                unsigned long long idle = v[3] + v[4];
                unsigned long long busy = total - idle;
                if (prev_total && total > prev_total)
                    g_telemetry_data.system.cpu_pct =
                        100.0f * (float)(busy - prev_busy) / (float)(total - prev_total);
                prev_busy = busy;
                prev_total = total;
            }
            else if (strncmp(line, "ctxt ", 5) == 0)
            {
                unsigned long long c = strtoull(line + 5, NULL, 10);
                if (prev_ctxt && dt > 0.0 && c >= prev_ctxt)
                    g_telemetry_data.system.ctxt_per_s =
                        (uint32_t)((c - prev_ctxt) / dt);
                prev_ctxt = c;
            }
            else if (strncmp(line, "procs_running ", 14) == 0)
                g_telemetry_data.system.procs_running =
                    (uint32_t)strtoul(line + 14, NULL, 10);
        }
        fclose(f);
    }

    /* this process's own CPU share */
    if ((f = fopen("/proc/self/stat", "r")))
    {
        char buf[1024];
        if (fgets(buf, sizeof(buf), f))
        {
            /* Field 14 (utime) and 15 (stime), 1-indexed. comm (field 2) can
             * contain spaces and parentheses, so start counting after the LAST
             * ')' rather than tokenising from the beginning. */
            char *p = strrchr(buf, ')');
            if (p)
            {
                unsigned long long ut = 0, st = 0;
                int field = 2;
                for (p++; *p && field < 14; p++)
                    if (*p == ' ')
                        field++;
                if (sscanf(p, "%llu %llu", &ut, &st) == 2 && hz > 0)
                {
                    unsigned long long self = ut + st;
                    if (prev_self && dt > 0.0 && self >= prev_self)
                        g_telemetry_data.system.bot_cpu_pct =
                            100.0f * (float)(self - prev_self) / (float)hz / (float)dt;
                    prev_self = self;
                }
            }
        }
        fclose(f);
    }

    if ((f = fopen("/proc/loadavg", "r")))
    {
        float l1 = 0.0f;
        if (fscanf(f, "%f", &l1) == 1)
            g_telemetry_data.system.load1 = l1;
        fclose(f);
    }

    if ((f = fopen("/proc/meminfo", "r")))
    {
        char line[256];
        unsigned long kb;
        while (fgets(line, sizeof(line), f))
        {
            if (sscanf(line, "MemTotal: %lu kB", &kb) == 1)
                g_telemetry_data.system.mem_total_kb = (uint32_t)kb;
            else if (sscanf(line, "MemAvailable: %lu kB", &kb) == 1)
            {
                g_telemetry_data.system.mem_avail_kb = (uint32_t)kb;
                break;      /* MemAvailable follows MemTotal; nothing else needed */
            }
        }
        fclose(f);
    }

    /* SoC temperature. Two things the previous version got wrong:
     *
     *  1. The path is not the same on every image. tools/bbot_watch.py already
     *     tries both spellings; this tried only one, so on an image carrying
     *     just the /sys/devices form the read silently never happened.
     *  2. A failed read left sys_temp_c at 0.0, which is indistinguishable
     *     from a real 0 °C. That is what the HUD has been showing.
     *
     * The probe is cached: a board with no thermal zone would otherwise pay
     * two failed opens every second, forever, to learn what it already knows. */
    {
        static const char *temp_path = NULL;
        static int temp_probed = 0;

        if (!temp_probed)
        {
            static const char *const candidates[] = {
                "/sys/class/thermal/thermal_zone0/temp",
                "/sys/devices/virtual/thermal/thermal_zone0/temp",
            };
            for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
            {
                if (access(candidates[i], R_OK) == 0)
                {
                    temp_path = candidates[i];
                    break;
                }
            }
            temp_probed = 1;
            if (temp_path)
                LOG_INFO("telemetry: SoC temperature from %s", temp_path);
            else
                LOG_INFO("telemetry: no thermal zone on this board, "
                         "SoC temperature reported as unavailable");
        }

        /* Reset every sample: a sensor that disappears must stop reporting the
         * last good value as if it were current. */
        g_telemetry_data.system.sys_temp_c = SYS_TEMP_NONE;
        if (temp_path && (f = fopen(temp_path, "r")))
        {
            long milli = 0;
            if (fscanf(f, "%ld", &milli) == 1)
                g_telemetry_data.system.sys_temp_c = milli / 1000.0f;
            fclose(f);
        }
    }

    /* The other bot processes. Matching on cmdline: "node" alone would match
     * any node process, and "python3" matches the OLED, serve_web.py and
     * anything else you happen to be running. */
    {
        static proc_watch_t w_node = {.needle = "server/server.js"};
        static proc_watch_t w_oled = {.needle = "bbb_oled.py"};
        static proc_watch_t w_batt = {.needle = "batt_monitor"};
        proc_watch_sample(&w_node, dt, hz);
        proc_watch_sample(&w_oled, dt, hz);
        proc_watch_sample(&w_batt, dt, hz);
        g_telemetry_data.system.node_alive = w_node.alive;
        g_telemetry_data.system.node_cpu_pct = w_node.cpu_pct;
        g_telemetry_data.system.oled_alive = w_oled.alive;
        g_telemetry_data.system.oled_cpu_pct = w_oled.cpu_pct;
        g_telemetry_data.system.batt_alive = w_batt.alive;
        g_telemetry_data.system.batt_cpu_pct = w_batt.cpu_pct;
    }
}

/**
 * @brief Update system status telemetry
 */
static void update_system_telemetry(void)
{
    if (!g_debug_config.telemetry.system_status)
        return;

    // Read battery voltage
    // Note: rc_adc_batt() returns battery voltage on BeagleBone Blue
    g_telemetry_data.system.battery_voltage = 0.0; //rc_adc_batt(); // fallback until batt_status read below

    // Robot state
    g_telemetry_data.system.armed = state.armed;
    g_telemetry_data.system.mode = state.mode;
    g_telemetry_data.system.theta_offset = state.theta_offset;

    // Read external battery monitor status (written by batt_monitor service)
    {
        /* A file much older than batt_monitor's write interval is not a
         * reading -- without this the dashboard showed the last voltage
         * forever after batt_monitor stopped. The limit must sit well above
         * that interval: batt_monitor runs --interval 60, and a 30 s limit
         * (first version of this) blanked BATT for half of every minute. */
        struct stat bst;
        FILE *f = NULL;
        if (stat("/run/batt_status.json", &bst) == 0 && time(NULL) - bst.st_mtime <= 150)
            f = fopen("/run/batt_status.json", "r");
        if (f)
        {
            char buf[128] = {0};
            fread(buf, 1, sizeof(buf) - 1, f);
            fclose(f);
            float v = 0.0f;
            char status[16] = "unknown";
            /* parse with strstr — immune to whitespace variations */
            const char *vp = strstr(buf, "\"voltage\":");
            if (vp)
                sscanf(vp, "\"voltage\": %f", &v);
            const char *sp = strstr(buf, "\"status\":");
            if (sp)
                sscanf(sp, "\"status\": \"%15[^\"]\"", status);
            g_telemetry_data.system.batt_voltage = v;
            g_telemetry_data.system.battery_voltage = v;
            if (strcmp(status, "critical") == 0)
                g_telemetry_data.system.batt_status = BATT_CRITICAL;
            else if (strcmp(status, "warning") == 0)
                g_telemetry_data.system.batt_status = BATT_WARNING;
            else
                g_telemetry_data.system.batt_status = BATT_OK;
        }
        else
        {
            g_telemetry_data.system.batt_voltage = -1.0f;
            g_telemetry_data.system.batt_status = BATT_UNKNOWN;
        }
    }

    // Uptime
    g_telemetry_data.system.uptime_sec = (uint32_t)(rc_nanos_since_boot() / 1000000000ULL);

    // Board health (CPU, load, memory, temperature, context switches) — 1 Hz
    update_board_health();

    // RoboClaw main battery voltage + temperature — polled at 1 Hz
    {
        static uint64_t last_claw_poll_us = 0;
        uint64_t now_us = rc_nanos_since_boot() / 1000;
        if (now_us - last_claw_poll_us >= 1000000ULL)
        {
            float v = 0.0f, t = 0.0f;
            if (motor_hal_read_voltage(&v) == 0)
                g_telemetry_data.system.claw_voltage = v;
            if (motor_hal_read_temp(&t) == 0)
                g_telemetry_data.system.claw_temp = t;
            last_claw_poll_us = now_us;
        }
    }

    /* Motor current — polled at 20 Hz, one serial round trip each.
     *
     * 20 Hz cannot see a millisecond edge and is not trying to. The RoboClaw's
     * own current limit handles fast events in hardware. What this is for is
     * the SUSTAINED draw — a stalled motor, a bot driving into the floor —
     * which lasts hundreds of milliseconds and therefore lands in several
     * consecutive samples. That is the load that heats wiring and connectors,
     * and it is the one a bench supply's averaging meter hides completely.
     *
     * The peaks are a running max, cleared from the dashboard, so a spike that
     * happened while you were looking away is still on screen when you look
     * back. */
    {
        static uint64_t last_amp_poll_us = 0;
        uint64_t now_us = rc_nanos_since_boot() / 1000;
        if (now_us - last_amp_poll_us >= 50000ULL)
        {
            float a1 = 0.0f, a2 = 0.0f;
            if (motor_hal_read_currents(&a1, &a2) == 0)
            {
                g_telemetry_data.system.claw_m1_amps = a1;
                g_telemetry_data.system.claw_m2_amps = a2;
                /* Track magnitude: a -6 A regen is as interesting as +6 A. */
                float m1 = fabsf(a1), m2 = fabsf(a2);
                if (m1 > g_telemetry_data.system.claw_m1_amps_peak)
                    g_telemetry_data.system.claw_m1_amps_peak = m1;
                if (m2 > g_telemetry_data.system.claw_m2_amps_peak)
                    g_telemetry_data.system.claw_m2_amps_peak = m2;
            }
            last_amp_poll_us = now_us;
        }
    }
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
void telemetry_update(void)
{
    // Update timestamp
    g_telemetry_data.timestamp_us = rc_nanos_since_boot() / 1000;

    // Update each telemetry component based on enable flags
    update_system_telemetry(); // Always update system status
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
int telemetry_init(void)
{
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
void telemetry_get_config_description(char *buffer, size_t size)
{
    int pos = 0;

    pos += snprintf(buffer + pos, size - pos, "Telemetry enabled: ");

    if (g_debug_config.telemetry.system_status)
        pos += snprintf(buffer + pos, size - pos, "system ");
    if (g_debug_config.telemetry.encoders)
        pos += snprintf(buffer + pos, size - pos, "encoders ");
    if (g_debug_config.telemetry.imu_attitude)
        pos += snprintf(buffer + pos, size - pos, "imu_attitude ");
    if (g_debug_config.telemetry.imu_full)
        pos += snprintf(buffer + pos, size - pos, "imu_full ");
    if (g_debug_config.telemetry.pid_states)
        pos += snprintf(buffer + pos, size - pos, "pid_states ");
    if (g_debug_config.telemetry.motor_commands)
        pos += snprintf(buffer + pos, size - pos, "motors ");
    if (g_debug_config.telemetry.ext_input)
        pos += snprintf(buffer + pos, size - pos, "ext_input ");

    if (pos == strlen("Telemetry enabled: "))
    {
        snprintf(buffer + pos, size - pos, "none");
    }
}

/**
 * @brief Print current telemetry to console
 *
 * Useful for debugging. Prints a human-readable summary of current telemetry.
 */
void telemetry_print_summary(void)
{
    printf("\n=== Telemetry Summary ===\n");
    printf("System: %.2fV %s Mode:%d %.1fHz\n",
           g_telemetry_data.system.battery_voltage,
           g_telemetry_data.system.armed ? "ARMED" : "DISARMED",
           g_telemetry_data.system.mode,
           g_telemetry_data.system.loop_hz);

    if (g_debug_config.telemetry.imu_attitude)
    {
        printf("IMU: θ=%.2f° φ=%.2f° ψ=%.2f°\n",
               g_telemetry_data.imu.theta,
               g_telemetry_data.imu.phi,
               g_telemetry_data.imu.psi);
        printf("QUAT: w=%.4f x=%.4f y=%.4f z=%.4f\n",
               g_telemetry_data.imu.qw,
               g_telemetry_data.imu.qx,
               g_telemetry_data.imu.qy,
               g_telemetry_data.imu.qz);
    }

    if (g_debug_config.telemetry.encoders)
    {
        printf("Encoders: L=%d(%.1fdeg,%.1fdeg/s) R=%d(%.1fdeg,%.1fdeg/s)\n",
               g_telemetry_data.encoders.left_ticks,
               g_telemetry_data.encoders.left_rad,
               g_telemetry_data.encoders.left_vel,
               g_telemetry_data.encoders.right_ticks,
               g_telemetry_data.encoders.right_rad,
               g_telemetry_data.encoders.right_vel);
    }

    if (g_debug_config.telemetry.pid_states)
    {
        printf("balance: err=%.3f out=%.3f %s\n",
               g_telemetry_data.pitch.error,
               g_telemetry_data.pitch.output,
               g_telemetry_data.pitch.enabled ? "ON" : "OFF");

        printf("steering: err=%.3f out=%.3f %s\n",
               g_telemetry_data.yaw.error,
               g_telemetry_data.yaw.output,
               g_telemetry_data.yaw.enabled ? "ON" : "OFF");
    }

    if (g_debug_config.telemetry.ext_input && g_telemetry_data.ext_input.valid)
    {
        printf("Cat: x=%.2f y=%.2f conf=%.0f%%\n",
               g_telemetry_data.ext_input.x,
               g_telemetry_data.ext_input.y,
               g_telemetry_data.ext_input.confidence * 100.0);
    }

    printf("========================\n\n");
}
