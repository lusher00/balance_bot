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
 * @file robot_config.c
 * @brief Single sectioned key=value config for every tunable on the robot.
 *
 * Replaces two files and four half-overlapping writers:
 *
 *   pidconfig.txt            positional lines + ad-hoc "# pos_config" section
 *   /etc/balance_bot_imu.conf   separate file, separate format, separate loader
 *
 * The positional format was actively dangerous — line 3 was the balance gains
 * and line 4 the steering gains, so deleting the unused D2 line was load-bearing
 * and inserting anything shifted every gain below it. Nothing in the file said so.
 *
 * The old writers were worse. pid_config_save() opened "w" and truncated;
 * pos_config_save() and motor_config_save() opened "a" and appended. They only
 * produced a correct file when called as that exact triple, in that exact order.
 * Call pos_config_save() on its own — as the set_pos_config IPC handler does —
 * and you appended a second [position] block on every write, forever.
 *
 * So: one struct, one reader, one writer. robot_config_save_current() snapshots
 * every live global and rewrites the whole file atomically. There is no way to
 * write a partial file and no way for two sections to disagree.
 */

#include "balance_bot.h"
#include "debug_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#define ROBOT_CONFIG_DEFAULT "robot.conf"

/* Files we migrate from, once, if robot.conf is absent. */
#define LEGACY_PID_FILE "pidconfig.txt"
#define LEGACY_IMU_FILE "/etc/balance_bot_imu.conf"
#define LEGACY_IMU_FILE_BACKUP "/home/debian/balance_bot_imu.conf"

/* ── helpers ──────────────────────────────────────────────────────────────── */

static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n'))
        *--e = '\0';
    return s;
}

/** Case-insensitive compare, so [Balance] and [balance] both work. */
static int ieq(const char *a, const char *b)
{
    for (; *a && *b; a++, b++)
    {
        char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
        if (ca != cb)
            return 0;
    }
    return *a == *b;
}

void robot_config_defaults(robot_config_t *c)
{
    memset(c, 0, sizeof(*c));

    c->balance.kp = BALANCE_KP;
    c->balance.ki = BALANCE_KI;
    c->balance.kd = BALANCE_KD;
    c->steering.kp = STEERING_KP;
    c->steering.ki = STEERING_KI;
    c->steering.kd = STEERING_KD;

    c->imu.pitch_offset = 0.0f;
    c->imu.yaw_offset = 0.0f;
    c->imu.pitch_dot_offset = 0.0f;
    c->imu.pitch_axis = 1;
    c->theta_trim = 0.0f;

    c->position.zone_a = POS_ZONE_A_DEFAULT;
    c->position.zone_b = POS_ZONE_B_DEFAULT;
    c->position.zone_c = POS_ZONE_C_DEFAULT;
    c->position.scale_a = POS_SCALE_A_DEFAULT;
    c->position.scale_b = POS_SCALE_B_DEFAULT;
    c->position.scale_c = POS_SCALE_C_DEFAULT;
    c->position.scale_d = POS_SCALE_D_DEFAULT;
    c->position.vel_scale_stop = POS_VEL_SCALE_STOP_DEFAULT;
    c->position.vel_scale_move = POS_VEL_SCALE_MOVE_DEFAULT;
    c->position.vel_scale_turning = POS_VEL_SCALE_TURNING_DEFAULT;
    c->position.stopped_vel = POS_STOPPED_VEL_DEFAULT;
    c->position.max_correction = POS_MAX_CORRECTION_DEFAULT;
    c->position.max_angle_rate = POS_MAX_ANGLE_RATE_DEFAULT;
    c->position.back_to_spot = POS_BACK_TO_SPOT_DEFAULT;

    c->motor.mode = MOTOR_HAL_MODE_DEFAULT;
    c->motor.qpps_max = MOTOR_QPPS_MAX_DEFAULT;
    c->motor.accel_qpps = MOTOR_ACCEL_QPPS_DEFAULT;
    c->motor.pol_l = 1.0f;
    c->motor.pol_r = 1.0f;
    c->motor.enc_pol_l = 1.0f;
    c->motor.enc_pol_r = 1.0f;
    c->motor.claw_kp = 0.0f;
    c->motor.claw_ki = 0.0f;
    c->motor.claw_kd = 0.0f;
    c->motor.baud = 460800;

    /* Deliberately gentle. MAX_THETA_REF is 17 deg; at scale 1.0 a full stick
     * asks for a 17 deg lean, which throws this chassis over. */
    c->sbus.drive_channel = 2;
    c->sbus.turn_channel = 1;
    c->sbus.drive_scale = 0.25f;
    c->sbus.turn_scale = 1.0f;
    c->sbus.turn_rate = 60.0f;
    c->sbus.drive_invert = 1;
    c->sbus.turn_invert = 0;
    c->sbus.deadband = 0.03f;
    c->sbus.require_center = 1;
}

/* ── read ─────────────────────────────────────────────────────────────────── */

int robot_config_load(const char *path, robot_config_t *c)
{
    if (!path)
        path = ROBOT_CONFIG_DEFAULT;

    FILE *f = fopen(path, "r");
    if (!f)
        return -1;

    char line[256], section[32] = "";
    int n = 0;

    while (fgets(line, sizeof(line), f))
    {
        char *s = trim(line);
        if (!*s || *s == '#' || *s == ';')
            continue;

        if (*s == '[')
        {
            char *end = strchr(s, ']');
            if (end)
            {
                *end = '\0';
                snprintf(section, sizeof(section), "%s", trim(s + 1));
            }
            continue;
        }

        char *eq = strchr(s, '=');
        if (!eq)
            continue;
        *eq = '\0';
        char *key = trim(s);
        char *val = trim(eq + 1);
        if (!*key || !*val)
            continue;
        float fv = strtof(val, NULL);
        n++;

#define KEY(k) (ieq(key, k))
        if (ieq(section, "balance"))
        {
            if (KEY("kp")) c->balance.kp = fv;
            else if (KEY("ki")) c->balance.ki = fv;
            else if (KEY("kd")) c->balance.kd = fv;
        }
        else if (ieq(section, "steering"))
        {
            if (KEY("kp")) c->steering.kp = fv;
            else if (KEY("ki")) c->steering.ki = fv;
            else if (KEY("kd")) c->steering.kd = fv;
        }
        else if (ieq(section, "imu"))
        {
            if (KEY("pitch_offset")) c->imu.pitch_offset = fv;
            else if (KEY("yaw_offset")) c->imu.yaw_offset = fv;
            else if (KEY("pitch_dot_offset")) c->imu.pitch_dot_offset = fv;
            else if (KEY("pitch_axis")) c->imu.pitch_axis = (int)fv;
            else if (KEY("theta_trim")) c->theta_trim = fv;
        }
        else if (ieq(section, "position"))
        {
            if (KEY("zone_a")) c->position.zone_a = (int32_t)fv;
            else if (KEY("zone_b")) c->position.zone_b = (int32_t)fv;
            else if (KEY("zone_c")) c->position.zone_c = (int32_t)fv;
            else if (KEY("scale_a")) c->position.scale_a = fv;
            else if (KEY("scale_b")) c->position.scale_b = fv;
            else if (KEY("scale_c")) c->position.scale_c = fv;
            else if (KEY("scale_d")) c->position.scale_d = fv;
            else if (KEY("vel_scale_stop")) c->position.vel_scale_stop = fv;
            else if (KEY("vel_scale_move")) c->position.vel_scale_move = fv;
            else if (KEY("vel_scale_turning")) c->position.vel_scale_turning = fv;
            else if (KEY("stopped_vel")) c->position.stopped_vel = (int32_t)fv;
            else if (KEY("max_correction")) c->position.max_correction = fv;
            else if (KEY("max_angle_rate")) c->position.max_angle_rate = fv;
            else if (KEY("back_to_spot")) c->position.back_to_spot = (int)fv;
        }
        else if (ieq(section, "sbus"))
        {
            if (KEY("drive_channel")) c->sbus.drive_channel = (int)fv;
            else if (KEY("turn_channel")) c->sbus.turn_channel = (int)fv;
            else if (KEY("drive_scale")) c->sbus.drive_scale = fv;
            else if (KEY("turn_scale")) c->sbus.turn_scale = fv;
            else if (KEY("turn_rate")) c->sbus.turn_rate = fv;
            else if (KEY("drive_invert")) c->sbus.drive_invert = (int)fv;
            else if (KEY("turn_invert")) c->sbus.turn_invert = (int)fv;
            else if (KEY("deadband")) c->sbus.deadband = fv;
            else if (KEY("require_center")) c->sbus.require_center = (int)fv;
        }
        else if (ieq(section, "motor"))
        {
            if (KEY("mode")) c->motor.mode = (int)fv;
            else if (KEY("qpps_max")) c->motor.qpps_max = (int)fv;
            else if (KEY("accel_qpps")) c->motor.accel_qpps = (int)fv;
            else if (KEY("pol_l")) c->motor.pol_l = fv;
            else if (KEY("pol_r")) c->motor.pol_r = fv;
            else if (KEY("enc_pol_l")) c->motor.enc_pol_l = fv;
            else if (KEY("enc_pol_r")) c->motor.enc_pol_r = fv;
            else if (KEY("claw_kp")) c->motor.claw_kp = fv;
            else if (KEY("claw_ki")) c->motor.claw_ki = fv;
            else if (KEY("claw_kd")) c->motor.claw_kd = fv;
            else if (KEY("baud")) c->motor.baud = (int)fv;
        }
#undef KEY
    }

    fclose(f);
    LOG_INFO("robot_config: loaded %d keys from %s", n, path);
    return 0;
}

/* ── write ────────────────────────────────────────────────────────────────── */

int robot_config_save(const char *path, const robot_config_t *c)
{
    if (!path)
        path = ROBOT_CONFIG_DEFAULT;

    /* Write to a temp file and rename. A power cut mid-write leaves the old
     * config intact rather than a truncated one — this file is the only copy
     * of a calibration that takes bench time to reproduce. */
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);

    FILE *f = fopen(tmp, "w");
    if (!f)
    {
        LOG_ERROR("robot_config_save: cannot open %s: %s", tmp, strerror(errno));
        return -1;
    }

    fprintf(f, "# robot.conf — balance_bot configuration\n");
    fprintf(f, "#\n");
    fprintf(f, "# Written by the firmware whenever anything is tuned. Hand-editing is\n");
    fprintf(f, "# fine; restart balance_bot to apply. Sections and keys are\n");
    fprintf(f, "# case-insensitive, order does not matter, unknown keys are ignored.\n");
    fprintf(f, "\n");

    fprintf(f, "# Balance loop: pitch angle -> motor duty. A real PID.\n");
    fprintf(f, "[balance]\n");
    fprintf(f, "kp = %.4f\n", c->balance.kp);
    fprintf(f, "ki = %.4f\n", c->balance.ki);
    fprintf(f, "kd = %.4f\n", c->balance.kd);
    fprintf(f, "\n");

    fprintf(f, "# Steering loop: wheel-rotation difference -> differential duty. A real PID.\n");
    fprintf(f, "[steering]\n");
    fprintf(f, "kp = %.4f\n", c->steering.kp);
    fprintf(f, "ki = %.4f\n", c->steering.ki);
    fprintf(f, "kd = %.4f\n", c->steering.kd);
    fprintf(f, "\n");

    fprintf(f, "# Position hold: encoder error -> lean-angle bias.\n");
    fprintf(f, "# NOT a PID — no gains. Zone-scheduled: |error| picks a zone, the\n");
    fprintf(f, "# correction is error/scale_N, then velocity-damped and clamped.\n");
    fprintf(f, "# scale_* are DIVISORS: larger means weaker correction.\n");
    fprintf(f, "[position]\n");
    fprintf(f, "zone_a            = %d\n", c->position.zone_a);
    fprintf(f, "zone_b            = %d\n", c->position.zone_b);
    fprintf(f, "zone_c            = %d\n", c->position.zone_c);
    fprintf(f, "scale_a           = %.3f\n", c->position.scale_a);
    fprintf(f, "scale_b           = %.3f\n", c->position.scale_b);
    fprintf(f, "scale_c           = %.3f\n", c->position.scale_c);
    fprintf(f, "scale_d           = %.3f\n", c->position.scale_d);
    fprintf(f, "vel_scale_stop    = %.3f\n", c->position.vel_scale_stop);
    fprintf(f, "vel_scale_move    = %.3f\n", c->position.vel_scale_move);
    fprintf(f, "vel_scale_turning = %.3f\n", c->position.vel_scale_turning);
    fprintf(f, "stopped_vel       = %d\n", c->position.stopped_vel);
    fprintf(f, "max_correction    = %.3f\n", c->position.max_correction);
    fprintf(f, "max_angle_rate    = %.3f\n", c->position.max_angle_rate);
    fprintf(f, "back_to_spot      = %d\n", c->position.back_to_spot);
    fprintf(f, "\n");

    fprintf(f, "# IMU mounting calibration. pitch_offset is the RAW pitch angle at\n");
    fprintf(f, "# which the robot actually balances — set by zero_imu, then trimmed.\n");
    fprintf(f, "# theta (deg) = pitch_offset - raw_pitch_deg\n");
    fprintf(f, "# Getting this wrong by a degree makes the bot creep and eats a fifth\n");
    fprintf(f, "# of the position hold's correction authority standing still.\n");
    fprintf(f, "[imu]\n");
    fprintf(f, "pitch_offset     = %.4f\n", c->imu.pitch_offset);
    fprintf(f, "yaw_offset       = %.4f\n", c->imu.yaw_offset);
    fprintf(f, "pitch_dot_offset = %.4f\n", c->imu.pitch_dot_offset);
    fprintf(f, "pitch_axis       = %d\n", c->imu.pitch_axis);
    fprintf(f, "theta_trim       = %.4f\n", c->theta_trim);
    fprintf(f, "\n");

    fprintf(f, "# Transmitter mapping. Channel numbers are 1-based, as shown on the TX.\n");
    fprintf(f, "#   drive_channel  CH2 (Ele) springs back to centre; CH3 (Thr) stays put.\n");
    fprintf(f, "#   drive_scale    multiplies the stick. MAX_THETA_REF is 17 deg, so 1.0\n");
    fprintf(f, "#                  asks for a 17 deg lean at full stick and will throw\n");
    fprintf(f, "#                  the bot over. Start low.\n");
    fprintf(f, "#   require_center drive stays 0 until the stick has been seen near\n");
    fprintf(f, "#                  centre. Matters for a ratcheted throttle, which rests\n");
    fprintf(f, "#                  at the bottom and would otherwise read as full reverse.\n");
    fprintf(f, "[sbus]\n");
    fprintf(f, "drive_channel  = %d\n", c->sbus.drive_channel);
    fprintf(f, "turn_channel   = %d\n", c->sbus.turn_channel);
    fprintf(f, "drive_scale    = %.3f\n", c->sbus.drive_scale);
    fprintf(f, "turn_scale     = %.3f\n", c->sbus.turn_scale);
    fprintf(f, "turn_rate      = %.1f\n", c->sbus.turn_rate);
    fprintf(f, "drive_invert   = %d\n", c->sbus.drive_invert);
    fprintf(f, "turn_invert    = %d\n", c->sbus.turn_invert);
    fprintf(f, "deadband       = %.3f\n", c->sbus.deadband);
    fprintf(f, "require_center = %d\n", c->sbus.require_center);
    fprintf(f, "\n");

    fprintf(f, "# RoboClaw drive stage. mode: 0=duty 1=velocity 2=velocity+accel\n");
    fprintf(f, "[motor]\n");
    fprintf(f, "mode       = %d\n", c->motor.mode);
    fprintf(f, "qpps_max   = %d\n", c->motor.qpps_max);
    fprintf(f, "accel_qpps = %d\n", c->motor.accel_qpps);
    fprintf(f, "pol_l      = %.1f\n", c->motor.pol_l);
    fprintf(f, "pol_r      = %.1f\n", c->motor.pol_r);
    fprintf(f, "enc_pol_l  = %.1f\n", c->motor.enc_pol_l);
    fprintf(f, "enc_pol_r  = %.1f\n", c->motor.enc_pol_r);
    fprintf(f, "baud       = %d\n", c->motor.baud);
    fprintf(f, "# RoboClaw internal velocity PID — only used in modes 1 and 2.\n");
    fprintf(f, "claw_kp    = %.4f\n", c->motor.claw_kp);
    fprintf(f, "claw_ki    = %.4f\n", c->motor.claw_ki);
    fprintf(f, "claw_kd    = %.4f\n", c->motor.claw_kd);

    fflush(f);
    fsync(fileno(f));
    fclose(f);

    if (rename(tmp, path) != 0)
    {
        LOG_ERROR("robot_config_save: rename %s -> %s failed: %s",
                  tmp, path, strerror(errno));
        return -1;
    }
    LOG_INFO("robot_config: saved %s", path);
    return 0;
}

/* ── live globals <-> struct ──────────────────────────────────────────────── */

void robot_config_get_current(robot_config_t *c)
{
    robot_config_defaults(c);
    c->balance.kp = balance_pid.kp;
    c->balance.ki = balance_pid.ki;
    c->balance.kd = balance_pid.kd;
    c->steering.kp = steering_pid.kp;
    c->steering.ki = steering_pid.ki;
    c->steering.kd = steering_pid.kd;
    c->position = g_pos_config;
    c->motor = g_motor_config;
    c->imu = g_imu_offsets;
    c->sbus = g_sbus_config;
    c->theta_trim = state.theta_offset;
}

void robot_config_apply(const robot_config_t *c)
{
    pid_set_gains(&balance_pid, c->balance.kp, c->balance.ki, c->balance.kd);
    pid_set_gains(&steering_pid, c->steering.kp, c->steering.ki, c->steering.kd);
    g_pos_config = c->position;
    g_motor_config = c->motor;
    g_imu_offsets = c->imu;
    g_sbus_config = c->sbus;
    state.theta_offset = c->theta_trim;
}

int robot_config_save_current(const char *path)
{
    robot_config_t c;
    robot_config_get_current(&c);
    return robot_config_save(path, &c);
}


/* ── the two apply helpers that outlived pid_config.c ─────────────────────── */

void pos_config_apply(const pos_config_t *cfg)
{
    g_pos_config = *cfg;
    LOG_INFO("pos_config applied: zones=%d/%d/%d scales=%.0f/%.0f/%.0f/%.0f "
             "vel=%.0f/%.0f/%.0f stopped=%d maxcorr=%.1f rate=%.1f bts=%d",
             cfg->zone_a, cfg->zone_b, cfg->zone_c,
             cfg->scale_a, cfg->scale_b, cfg->scale_c, cfg->scale_d,
             cfg->vel_scale_stop, cfg->vel_scale_move, cfg->vel_scale_turning,
             cfg->stopped_vel, cfg->max_correction,
             cfg->max_angle_rate, cfg->back_to_spot);
}

void motor_config_apply(const motor_config_t *cfg)
{
    g_motor_config = *cfg;
    LOG_INFO("motor_config applied: mode=%d qpps_max=%d accel_qpps=%d pol_l=%.1f pol_r=%.1f "
             "enc_pol_l=%.1f enc_pol_r=%.1f claw_kp=%.4f claw_ki=%.4f claw_kd=%.4f",
             cfg->mode, cfg->qpps_max, cfg->accel_qpps, cfg->pol_l, cfg->pol_r,
             cfg->enc_pol_l, cfg->enc_pol_r, cfg->claw_kp, cfg->claw_ki, cfg->claw_kd);
}

/* ── migration ────────────────────────────────────────────────────────────── */

/**
 * Read the old positional pidconfig.txt. Layout, which existed only in comments:
 *   line 1: legacy holdPosition flag, unused
 *   line 2: balance_angle / theta trim
 *   line 3: balance  kp ki kd
 *   line 4: steering kp ki kd
 *   then "# pos_config" and "# motor_config" key=value sections
 */
static int load_legacy_pid(const char *path, robot_config_t *c)
{
    FILE *f = fopen(path, "r");
    if (!f)
        return -1;

    int dummy;
    if (fscanf(f, "%d", &dummy) != 1) { fclose(f); return -1; }
    if (fscanf(f, "%f", &c->theta_trim) != 1) { fclose(f); return -1; }
    if (fscanf(f, "%f %f %f", &c->balance.kp, &c->balance.ki, &c->balance.kd) != 3)
    { fclose(f); return -1; }
    if (fscanf(f, "%f %f %f", &c->steering.kp, &c->steering.ki, &c->steering.kd) != 3)
    { fclose(f); return -1; }

    /* The trailing key=value sections are section-agnostic in the old format —
     * keys are unique across pos_config and motor_config, so a flat sweep is
     * safe and avoids depending on the "# pos_config" marker being present. */
    char line[256];
    while (fgets(line, sizeof(line), f))
    {
        char *s = trim(line);
        if (!*s || *s == '#')
            continue;
        char *eq = strchr(s, '=');
        if (!eq)
            continue;
        *eq = '\0';
        char *k = trim(s);
        float v = strtof(trim(eq + 1), NULL);
        if (ieq(k, "zone_a")) c->position.zone_a = (int32_t)v;
        else if (ieq(k, "zone_b")) c->position.zone_b = (int32_t)v;
        else if (ieq(k, "zone_c")) c->position.zone_c = (int32_t)v;
        else if (ieq(k, "scale_a")) c->position.scale_a = v;
        else if (ieq(k, "scale_b")) c->position.scale_b = v;
        else if (ieq(k, "scale_c")) c->position.scale_c = v;
        else if (ieq(k, "scale_d")) c->position.scale_d = v;
        else if (ieq(k, "vel_scale_stop")) c->position.vel_scale_stop = v;
        else if (ieq(k, "vel_scale_move")) c->position.vel_scale_move = v;
        else if (ieq(k, "vel_scale_turning")) c->position.vel_scale_turning = v;
        else if (ieq(k, "stopped_vel")) c->position.stopped_vel = (int32_t)v;
        else if (ieq(k, "max_correction")) c->position.max_correction = v;
        else if (ieq(k, "max_angle_rate")) c->position.max_angle_rate = v;
        else if (ieq(k, "back_to_spot")) c->position.back_to_spot = (int)v;
        else if (ieq(k, "mode")) c->motor.mode = (int)v;
        else if (ieq(k, "qpps_max")) c->motor.qpps_max = (int)v;
        else if (ieq(k, "accel_qpps")) c->motor.accel_qpps = (int)v;
        else if (ieq(k, "pol_l")) c->motor.pol_l = v;
        else if (ieq(k, "pol_r")) c->motor.pol_r = v;
        else if (ieq(k, "enc_pol_l")) c->motor.enc_pol_l = v;
        else if (ieq(k, "enc_pol_r")) c->motor.enc_pol_r = v;
        else if (ieq(k, "claw_kp")) c->motor.claw_kp = v;
        else if (ieq(k, "claw_ki")) c->motor.claw_ki = v;
        else if (ieq(k, "claw_kd")) c->motor.claw_kd = v;
        else if (ieq(k, "baud")) c->motor.baud = (int)v;
    }
    fclose(f);
    return 0;
}

/** Old IMU file: bare "key value" pairs, space separated, no sections. */
static int load_legacy_imu(const char *path, robot_config_t *c)
{
    FILE *f = fopen(path, "r");
    if (!f)
        return -1;
    char k[64];
    float v;
    int n = 0;
    while (fscanf(f, "%63s %f", k, &v) == 2)
    {
        if (ieq(k, "pitch_offset")) { c->imu.pitch_offset = v; n++; }
        else if (ieq(k, "yaw_offset")) { c->imu.yaw_offset = v; n++; }
        else if (ieq(k, "pitch_dot_offset")) { c->imu.pitch_dot_offset = v; n++; }
        else if (ieq(k, "pitch_axis")) { c->imu.pitch_axis = (int)v; n++; }
    }
    fclose(f);
    return n > 0 ? 0 : -1;
}

int robot_config_load_or_migrate(const char *path, robot_config_t *c)
{
    if (!path)
        path = ROBOT_CONFIG_DEFAULT;

    robot_config_defaults(c);

    if (robot_config_load(path, c) == 0)
        return 0;

    /* No robot.conf. Pull whatever the old files hold and write the new one.
     * Losing pitch_offset here means an hour on the bench, so this is loud
     * about what it found and what it did not. */
    LOG_WARN("robot_config: %s not found — migrating from legacy files", path);

    int got_pid = (load_legacy_pid(LEGACY_PID_FILE, c) == 0);
    LOG_WARN("  %s: %s", LEGACY_PID_FILE, got_pid ? "migrated" : "absent/unreadable");

    int got_imu = (load_legacy_imu(LEGACY_IMU_FILE, c) == 0);
    if (!got_imu)
        got_imu = (load_legacy_imu(LEGACY_IMU_FILE_BACKUP, c) == 0);
    LOG_WARN("  IMU offsets: %s (pitch_offset=%.4f)",
             got_imu ? "migrated" : "NOT FOUND — recalibrate with zero_imu",
             c->imu.pitch_offset);

    if (!got_pid && !got_imu)
    {
        LOG_WARN("robot_config: nothing to migrate, using compile-time defaults");
    }

    if (robot_config_save(path, c) == 0)
    {
        char cwd[512];
        LOG_WARN("robot_config: wrote %s (cwd %s) — the legacy files are now unused",
                 path, getcwd(cwd, sizeof cwd) ? cwd : "?");
    }
    else
    {
        /* Silence here was a trap: the migration would report success, the robot
         * would fly on values held only in memory, and the next restart would
         * migrate from scratch again. If the path is relative it resolves against
         * the service's WorkingDirectory, which is a common reason this fails. */
        char cwd[512];
        LOG_ERROR("robot_config: FAILED to write %s (cwd %s): %s — "
                  "config is in memory only and will NOT survive a restart",
                  path, getcwd(cwd, sizeof cwd) ? cwd : "?", strerror(errno));
    }

    return 0;
}
