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
 * @file motor_hal_roboclaw.c
 * @brief Motor HAL backend: RoboClaw via roboclaw library (Bartosz Meglicki).
 *
 * Uses roboclaw_init() / roboclaw_speed_m1m2() / roboclaw_duty_m1m2() / roboclaw_encoders()
 * from the vendored roboclaw library (src/roboclaw.c + include/roboclaw.h).
 * That library owns the tty directly via termios — do NOT also open it
 * through rc_uart.
 *
 * To activate: in the Makefile set
 *   MOTOR_HAL = src/motor_hal_roboclaw.c
 * and pass -m /dev/ttyOx [-B <baud>] on the command line.
 *
 * Wiring notes:
 *   M1 = RIGHT motor  (negate g_motor_config.pol_r if it runs backwards)
 *   M2 = LEFT  motor  (negate g_motor_config.pol_l if it runs backwards)
 *   Address 0x80 is the RoboClaw factory default.
 *
 * ── Drive mode ───────────────────────────────────────────────────────────────
 * All drive parameters are runtime-tunable via the IPC set_motor_config command
 * and live in g_motor_config (balance_bot.h / pid_config.c).
 *
 *   mode 0  MOTOR_HAL_MODE_DUTY          — raw PWM duty (MIXEDDUTY, cmd 34)
 *   mode 1  MOTOR_HAL_MODE_VELOCITY      — closed-loop QPPS (MIXEDSPEED, cmd 37)
 *   mode 2  MOTOR_HAL_MODE_VELOCITY_ACCEL— closed-loop QPPS + ramp (cmd 40)
 *
 * For velocity modes the HAL maps the normalised ±1.0f input to
 * ±g_motor_config.qpps_max encoder pulses/second.
 *
 * Important: velocity modes require the RoboClaw encoder inputs to be wired
 * and the velocity PID tuned in Basic Micro Motion Studio before use.
 * The RoboClaw velocity PID is completely separate from balance_bot's PID.
 */

#include "motor_hal.h"
#include "balance_bot.h"
#include "roboclaw.h"
#include "roboclaw_estop.h"
#include "roboclaw_estop.h"
#include <string.h>
#include <stdint.h>
#include <pthread.h>

/* ── wiring ──────────────────────────────────────────────────────────────── */
#define RC_ADDRESS 0x80 /* confirmed address for this robot */

/* duty range the RoboClaw expects for MIXEDDUTY (cmd 34): -32767 .. +32767 */
#define DUTY_MAX 32767

static struct roboclaw *g_rc = NULL;
static pthread_mutex_t g_rc_mutex = PTHREAD_MUTEX_INITIALIZER;
static char g_device[64] = "/dev/ttyS1";
static int g_baud = 460800;

/* Bench/stub mode: no RoboClaw wired up. Selected with -m none.
 * Every function in this file already guards on g_rc, so the stub only has to
 * suppress the -1 returns that would otherwise be produced 100 times a second
 * by a control loop that is working exactly as intended. */
static int g_stub = 0;

/* last encoder values — cached so motor_hal_encoder_read() works per-side */
static int32_t g_enc_l = 0;
static int32_t g_enc_r = 0;

/* ── Link watchdog ──────────────────────────────────────────────────────────
 * The library blocks: at 460800 baud it waits 5 ms for a reply and retries 3
 * times, so ONE command to a RoboClaw that is not answering costs 15 ms. The
 * control loop issues a duty write plus an encoder read every tick and a speed
 * poll every 40 ms, which is 30-60 ms of pure timeout per tick -- the 100 Hz
 * loop falls to 16-33 Hz. Switching the controller off, or a loose connector,
 * silently destroys the control loop rather than just stopping the motors.
 *
 * So: count consecutive failures. After LINK_FAIL_TRIP of them, declare the
 * link down and return immediately from every call without touching the port,
 * probing only once every LINK_RETRY_MS. A dead RoboClaw then costs one
 * timeout every half second instead of six per tick, and the loop holds rate.
 * Any success clears it. */
#define LINK_FAIL_TRIP   3
#define LINK_RETRY_MS    500

static int      g_link_fails = 0;
static int      g_link_down  = 0;
static uint64_t g_link_next_probe_ms = 0;

static uint64_t now_ms(void)
{
    return rc_nanos_since_boot() / 1000000ULL;
}

/* True when the caller should skip the port entirely. Lets one probe through
 * every LINK_RETRY_MS so the link can come back on its own when the controller
 * is switched on again -- no restart needed. */
static int link_blocked(void)
{
    if (!g_link_down)
        return 0;
    uint64_t t = now_ms();
    if (t >= g_link_next_probe_ms)
    {
        g_link_next_probe_ms = t + LINK_RETRY_MS;
        return 0; /* let this one through as the probe */
    }
    return 1;
}

static void link_result(int ok)
{
    if (ok)
    {
        if (g_link_down)
            LOG_WARN("motor_hal: RoboClaw link restored");
        g_link_down = 0;
        g_link_fails = 0;
        return;
    }
    if (g_link_down)
        return;
    if (++g_link_fails >= LINK_FAIL_TRIP)
    {
        g_link_down = 1;
        g_link_next_probe_ms = now_ms() + LINK_RETRY_MS;
        LOG_WARN("motor_hal: RoboClaw not answering — link marked down, "
                 "skipping serial I/O to protect the control loop rate "
                 "(probing every %d ms)", LINK_RETRY_MS);
    }
}

int motor_hal_link_down(void) { return g_link_down; }

/* ── helpers ────────────────────────────────────────────────────── */

static int refresh_encoders(void)
{
    if (!g_rc)
        return -1;
    if (link_blocked())
        return -1;
    int32_t m1, m2;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_encoders(g_rc, RC_ADDRESS, &m1, &m2);
    pthread_mutex_unlock(&g_rc_mutex);
    link_result(ret == ROBOCLAW_OK);
    if (ret != ROBOCLAW_OK)
    {
        /* 100 Hz path, x2 for both wheels. A RoboClaw that fails one read
         * fails the next as well, so this is a ~200 Hz writer exactly when
         * the link is already in trouble. */
        LOG_WARN_EVERY(2000, "motor_hal_roboclaw: encoder read failed (%d)", ret);
        return -1;
    }
    // Apply per-motor encoder polarity from motor_config (runtime tunable via IPC)
    g_enc_r = (int32_t)(g_motor_config.enc_pol_r * (float)m1);
    g_enc_l = (int32_t)(g_motor_config.enc_pol_l * (float)m2);
    return 0;
}

/* ── init / cleanup ─────────────────────────────────────────────── */

int motor_hal_init(const char *device, int baud)
{
    if (device && (strcmp(device, "none") == 0 ||
                   strcmp(device, "stub") == 0 ||
                   strcmp(device, "sim")  == 0))
    {
        g_stub  = 1;
        g_rc    = NULL;
        g_enc_l = 0;
        g_enc_r = 0;
        strncpy(g_device, "none", sizeof(g_device) - 1);
        g_baud = (baud > 0) ? baud : 460800;
        LOG_WARN("motor_hal_roboclaw: STUB MODE (-m none) — no RoboClaw, "
                 "motors will not move, encoders read 0");
        return 0;
    }

    if (!device)
        device = "/dev/ttyS1";
    if (baud <= 0)
        baud = 460800;
    strncpy(g_device, device, sizeof(g_device) - 1);
    g_baud = baud;

    LOG_INFO("motor_hal_roboclaw: opening %s at %d baud (addr 0x%02X)",
             device, baud, RC_ADDRESS);

    g_rc = roboclaw_init(device, baud);
    if (!g_rc)
    {
        LOG_ERROR("motor_hal_roboclaw: roboclaw_init() failed — check device and baud");
        return -1;
    }

    roboclaw_estop_init();

    /* Coast, not set_both(0,0): in velocity mode the latter is the FIRST thing
     * that ever commands this controller, and it commands a closed loop to hold
     * zero rather than leaving the motors unpowered. */
    motor_hal_coast();

    /* Push the current limit before anything can command motion. This is the
     * real overcurrent protection: it acts in hardware in microseconds, where
     * a fuse acts on I2t over seconds and a capacitor covers microseconds only. */
    if (g_motor_config.max_amps > 0.0f)
        motor_hal_set_current_limit(g_motor_config.max_amps);

    if (roboclaw_reset_encoders(g_rc, RC_ADDRESS) != ROBOCLAW_OK)
        LOG_WARN("motor_hal_roboclaw: encoder reset at init failed");
    else
        LOG_INFO("motor_hal_roboclaw: encoders zeroed");

    LOG_INFO("motor_hal_roboclaw: ready");
    return 0;
}

void motor_hal_cleanup(void)
{
    if (!g_rc)
        return;
    motor_hal_coast();      /* off, not a velocity hold — see motor_hal_coast */
    roboclaw_close(g_rc);
    g_rc = NULL;
    LOG_INFO("motor_hal_roboclaw: cleaned up");
}

int motor_hal_roboclaw_reset(void)
{
    if (g_stub)
        return 0;
    LOG_INFO("motor_hal_roboclaw: resetting RoboClaw (WriteNVM)...");
    pthread_mutex_lock(&g_rc_mutex);
    if (g_rc)
    {
        roboclaw_duty_m1m2(g_rc, RC_ADDRESS, 0, 0);
        roboclaw_close(g_rc);
        g_rc = NULL;
    }
    pthread_mutex_unlock(&g_rc_mutex);
    roboclaw_estop_deassert();
    usleep(50000);
    int ret = system("python3 /home/debian/balance_bot/roboclaw_reset.py");
    if (ret != 0)
        LOG_WARN("motor_hal_roboclaw: reset script exited %d — continuing", ret);
    pthread_mutex_lock(&g_rc_mutex);
    g_rc = roboclaw_init(g_device, g_baud);
    pthread_mutex_unlock(&g_rc_mutex);
    if (!g_rc)
    {
        LOG_ERROR("motor_hal_roboclaw: reinit failed after reset");
        return -1;
    }
    LOG_INFO("motor_hal_roboclaw: reset complete, e-stop cleared");
    return 0;
}

/* ── motor output ───────────────────────────────────────────────── */

int motor_hal_set_both(float left, float right)
{
    if (g_stub)
        return 0;
    if (!g_rc)
        return -1;

    /* clamp to ±1.0 */
    if (left > 1.0f)
        left = 1.0f;
    if (left < -1.0f)
        left = -1.0f;
    if (right > 1.0f)
        right = 1.0f;
    if (right < -1.0f)
        right = -1.0f;

    const float pol_l = g_motor_config.pol_l;
    const float pol_r = g_motor_config.pol_r;
    const int mode = g_motor_config.mode;
    const int qpps_max = g_motor_config.qpps_max;
    const int accel = g_motor_config.accel_qpps;

    /* Motors are already stopped when the link is down -- the controller's own
     * serial timeout kills output within tens of ms of it stopping hearing from
     * us, which is exactly the behaviour we want. Nothing is gained by spending
     * 15 ms per tick discovering that again. */
    if (link_blocked())
        return -1;

    int ret;
    pthread_mutex_lock(&g_rc_mutex);

    if (mode == MOTOR_HAL_MODE_VELOCITY_ACCEL)
    {
        /* closed-loop velocity + acceleration ramp (MIXEDSPEEDACCEL, cmd 40) */
        int32_t spd_r = (int32_t)(pol_r * right * (float)qpps_max);
        int32_t spd_l = (int32_t)(pol_l * left * (float)qpps_max);
        ret = roboclaw_speed_accel_m1m2(g_rc, RC_ADDRESS, spd_r, spd_l, accel);
        if (ret != ROBOCLAW_OK)
            /* Written once per control tick while driving. */
            LOG_WARN_EVERY(2000, "motor_hal_roboclaw: speed_accel command failed (%d)", ret);
    }
    else if (mode == MOTOR_HAL_MODE_VELOCITY)
    {
        /* closed-loop velocity, no ramp (MIXEDSPEED, cmd 37) */
        int32_t spd_r = (int32_t)(pol_r * right * (float)qpps_max);
        int32_t spd_l = (int32_t)(pol_l * left * (float)qpps_max);
        ret = roboclaw_speed_m1m2(g_rc, RC_ADDRESS, spd_r, spd_l);
        if (ret != ROBOCLAW_OK)
            /* Per control tick in velocity mode — throttled for the same
             * reason as speed_accel above. */
            LOG_WARN_EVERY(2000, "motor_hal_roboclaw: speed command failed (%d)", ret);
    }
    else
    {
        /* raw PWM duty cycle (MIXEDDUTY, cmd 34) — default, no encoder feedback */
        int16_t d1 = (int16_t)(pol_r * right * (float)DUTY_MAX);
        int16_t d2 = (int16_t)(pol_l * left * (float)DUTY_MAX);
        ret = roboclaw_duty_m1m2(g_rc, RC_ADDRESS, d1, d2);
        if (ret != ROBOCLAW_OK)
            /* Per control tick in duty mode. */
            LOG_WARN_EVERY(2000, "motor_hal_roboclaw: duty command failed (%d)", ret);
    }

    pthread_mutex_unlock(&g_rc_mutex);
    link_result(ret == ROBOCLAW_OK);
    return (ret == ROBOCLAW_OK) ? 0 : -1;
}

int motor_hal_set(int motor, float duty)
{
    /* roboclaw_duty_m1m2 drives both motors in one packet — keep the
     * other side at zero.  If you need independent control use set_both. */
    return motor == MOTOR_LEFT
               ? motor_hal_set_both(duty, 0.0f)
               : motor_hal_set_both(0.0f, duty);
}

int motor_hal_free_spin(void)
{
    /* Free spin means UNPOWERED. In velocity mode set_both(0,0) was the exact
     * opposite: a closed loop actively resisting any attempt to turn the
     * wheels by hand. */
    return motor_hal_coast();
}

int motor_hal_coast(void)
{
    if (g_stub)
        return 0;
    if (!g_rc)
        return -1;
    if (link_blocked())
        return -1;

    /* MIXEDDUTY 0 regardless of g_motor_config.mode. See the header for why
     * this cannot go through motor_hal_set_both(): in velocity mode a zero
     * there is a closed-loop request to HOLD zero, not an absence of drive. */
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_duty_m1m2(g_rc, RC_ADDRESS, 0, 0);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
        LOG_WARN_EVERY(2000, "motor_hal_roboclaw: coast (duty 0) failed (%d)", ret);
    link_result(ret == ROBOCLAW_OK);
    return (ret == ROBOCLAW_OK) ? 0 : -1;
}

int motor_hal_standby(int standby)
{
    /* RoboClaw has no standby pin. Cutting power is the equivalent -- NOT
     * commanding zero speed, which is a live controller holding a setpoint. */
    if (standby)
        return motor_hal_coast();
    return 0;
}

/* ── encoder input ──────────────────────────────────────────────── */

int32_t motor_hal_encoder_read(int motor)
{
    /* Refresh both encoders in a single UART transaction.
     * Guard with a timestamp so back-to-back left/right calls in the
     * same loop tick only hit the wire once. */
    static uint64_t last_refresh_us = 0;
    uint64_t now_us = rc_nanos_since_boot() / 1000;
    if (now_us - last_refresh_us > 5000)
    { /* refresh at most every 5 ms */
        refresh_encoders();
        last_refresh_us = now_us;
    }
    return (motor == MOTOR_LEFT) ? g_enc_l : g_enc_r;
}

int motor_hal_encoder_reset(int motor)
{
    /* The library exposes RESETENC (cmd 20) only via roboclaw_encoders —
     * no per-motor reset in the current API.  Reset both and zero cache. */
    (void)motor;
    return motor_hal_encoder_reset_all();
}

int motor_hal_encoder_reset_all(void)
{
    g_enc_l = 0;
    g_enc_r = 0;
    if (g_stub)
        return 0;
    if (!g_rc)
        return -1;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_reset_encoders(g_rc, RC_ADDRESS);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
        LOG_WARN("motor_hal_roboclaw: hardware encoder reset failed (%d)", ret);
    return (ret == ROBOCLAW_OK) ? 0 : -1;
}

/* ── RoboClaw velocity PID ──────────────────────────────────────── */

int motor_hal_set_claw_pid(float kp, float ki, float kd)
{
    if (g_stub)
        return 0;
    if (!g_rc)
        return -1;
    roboclaw_vel_pid_t pid = {.kp = kp, .ki = ki, .kd = kd};
    uint32_t qpps = (uint32_t)g_motor_config.qpps_max;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_set_velocity_pid(g_rc, RC_ADDRESS, &pid, qpps);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
    {
        LOG_WARN("motor_hal_set_claw_pid: failed (%d) kp=%.4f ki=%.4f kd=%.4f", ret, kp, ki, kd);
        return -1;
    }
    /* Keep g_motor_config in sync so save_pid persists the values */
    g_motor_config.claw_kp = kp;
    g_motor_config.claw_ki = ki;
    g_motor_config.claw_kd = kd;
    LOG_INFO("Claw velocity PID: kp=%.4f ki=%.4f kd=%.4f qpps=%u", kp, ki, kd, qpps);
    return 0;
}

/* ── Reading the controller's own settings ──────────────────────────
 * Six round trips at ~1 ms each.  Called from the config path, never from the
 * control loop.  All-or-nothing: a partial read is worse than none, because a
 * half-filled struct on screen looks authoritative. */

int motor_hal_read_hw_settings(claw_hw_settings_t *out)
{
    if (!out)
        return -1;
    memset(out, 0, sizeof(*out));
    if (g_stub || !g_rc)
        return -1;

    roboclaw_vel_pid_t p1 = {0}, p2 = {0};
    uint32_t q1 = 0, q2 = 0;
    uint8_t em1 = 0, em2 = 0;
    uint16_t cfg = 0;
    int ret;

    pthread_mutex_lock(&g_rc_mutex);
    ret = roboclaw_read_velocity_pid(g_rc, RC_ADDRESS, 0, &p1, &q1);
    if (ret == ROBOCLAW_OK)
        ret = roboclaw_read_velocity_pid(g_rc, RC_ADDRESS, 1, &p2, &q2);
    if (ret == ROBOCLAW_OK)
        ret = roboclaw_read_encoder_mode(g_rc, RC_ADDRESS, &em1, &em2);
    if (ret == ROBOCLAW_OK)
        ret = roboclaw_read_config(g_rc, RC_ADDRESS, &cfg);
    pthread_mutex_unlock(&g_rc_mutex);

    if (ret != ROBOCLAW_OK)
    {
        LOG_WARN("motor_hal_read_hw_settings: read failed (%d)", ret);
        return -1;
    }

    out->m1_kp = p1.kp; out->m1_ki = p1.ki; out->m1_kd = p1.kd; out->m1_qpps = q1;
    out->m2_kp = p2.kp; out->m2_ki = p2.ki; out->m2_kd = p2.kd; out->m2_qpps = q2;
    out->enc_mode_m1 = em1;
    out->enc_mode_m2 = em2;
    out->config = cfg;
    out->valid = 1;
    return 0;
}

/* ── Battery voltage ────────────────────────────────────────────── */

int motor_hal_read_voltage(float *volts)
{
    *volts = 0.0f;
    if (!g_rc)
        return -1;
    int16_t raw = 0;
    if (link_blocked())
        return -1;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_main_battery_voltage(g_rc, RC_ADDRESS, &raw);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
        return -1;
    /* RoboClaw returns tenths of a volt (e.g. 118 = 11.8 V) */
    *volts = raw / 10.0f;
    return 0;
}

/* ── Encoder speed ──────────────────────────────────────────────── */

int motor_hal_read_encoder_speeds(int32_t *m1_qpps, int32_t *m2_qpps)
{
    *m1_qpps = 0;
    *m2_qpps = 0;
    if (!g_rc)
        return -1;
    if (link_blocked())
        return -1;
    int32_t m1 = 0, m2 = 0;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_encoder_speeds(g_rc, RC_ADDRESS, &m1, &m2);
    pthread_mutex_unlock(&g_rc_mutex);
    link_result(ret == ROBOCLAW_OK);
    if (ret != ROBOCLAW_OK)
        return -1;
    /* Apply encoder polarity to match position encoder convention */
    *m1_qpps = (int32_t)(g_motor_config.enc_pol_r * (float)m1);
    *m2_qpps = (int32_t)(g_motor_config.enc_pol_l * (float)m2);
    return 0;
}

/* ── Temperature ────────────────────────────────────────────────── */

int motor_hal_read_currents(float *m1_amps, float *m2_amps)
{
    *m1_amps = 0.0f;
    *m2_amps = 0.0f;
    if (g_stub || !g_rc)
        return -1;
    if (link_blocked())
        return -1;
    float a1 = 0.0f, a2 = 0.0f;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_currents(g_rc, RC_ADDRESS, &a1, &a2);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
        return -1;
    /* M1 = RIGHT, M2 = LEFT (see the header) — reported in that order so the
     * labels line up with the encoder convention used everywhere else. */
    *m1_amps = a1;
    *m2_amps = a2;
    return 0;
}

int motor_hal_set_current_limit(float amps)
{
    if (amps <= 0.0f)
        return 0; /* leave whatever the controller already has */
    if (g_stub)
    {
        g_motor_config.max_amps = amps;
        return 0;
    }
    if (!g_rc)
        return -1;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_set_max_current(g_rc, RC_ADDRESS, 0, amps);
    if (ret == ROBOCLAW_OK)
        ret = roboclaw_set_max_current(g_rc, RC_ADDRESS, 1, amps);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
    {
        LOG_WARN("motor_hal_set_current_limit: failed (%d) at %.2f A", ret, amps);
        return -1;
    }
    g_motor_config.max_amps = amps;
    LOG_INFO("RoboClaw current limit: %.2f A per motor (%.2f A total)", amps, amps * 2.0f);
    return 0;
}

int motor_hal_read_current_limit(float *amps)
{
    *amps = 0.0f;
    if (g_stub || !g_rc)
        return -1;
    if (link_blocked())
        return -1;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_read_max_current(g_rc, RC_ADDRESS, 0, amps);
    pthread_mutex_unlock(&g_rc_mutex);
    return (ret == ROBOCLAW_OK) ? 0 : -1;
}

int motor_hal_read_temp(float *temp_c)
{
    *temp_c = 0.0f;
    if (!g_rc)
        return -1;
    if (link_blocked())
        return -1;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_temperature(g_rc, RC_ADDRESS, temp_c);
    pthread_mutex_unlock(&g_rc_mutex);
    return (ret == ROBOCLAW_OK) ? 0 : -1;
}

/* ── Baud rate change ───────────────────────────────────────────── */

int motor_hal_set_baud(int baud)
{
    if (baud <= 0)
        return -1;
    if (g_stub)
    {
        g_baud = baud;
        return 0;
    }
    pthread_mutex_lock(&g_rc_mutex);
    /* Stop motors before disconnecting */
    if (g_rc)
    {
        roboclaw_duty_m1m2(g_rc, RC_ADDRESS, 0, 0);
        roboclaw_close(g_rc);
        g_rc = NULL;
    }
    g_baud = baud;
    g_motor_config.baud = baud;
    g_rc = roboclaw_init(g_device, g_baud);
    pthread_mutex_unlock(&g_rc_mutex);
    if (!g_rc)
    {
        LOG_ERROR("motor_hal_set_baud: reconnect at %d baud failed", baud);
        return -1;
    }
    LOG_INFO("motor_hal_set_baud: reconnected at %d baud", baud);
    return 0;
}