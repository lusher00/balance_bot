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

#include <string.h>
#include <stdint.h>
#include <pthread.h>

/* ── wiring ──────────────────────────────────────────────────────────────── */
#define RC_ADDRESS 0x80   /* default RoboClaw address */

/* duty range the RoboClaw expects for MIXEDDUTY (cmd 34): -32767 .. +32767 */
#define DUTY_MAX 32767

static struct roboclaw *g_rc = NULL;
static pthread_mutex_t g_rc_mutex = PTHREAD_MUTEX_INITIALIZER;

/* last encoder values — cached so motor_hal_encoder_read() works per-side */
static int32_t g_enc_l = 0;
static int32_t g_enc_r = 0;

/* ── helpers ────────────────────────────────────────────────────── */

static int refresh_encoders(void)
{
    if (!g_rc)
        return -1;
    int32_t m1, m2;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_encoders(g_rc, RC_ADDRESS, &m1, &m2);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
    {
        LOG_WARN("motor_hal_roboclaw: encoder read failed (%d)", ret);
        return -1;
    }
    // M2 = left wheel, positive when moving forward (M2 & ENC2 are negated in basic micro motion studio)
    // M1 = right wheel, positive when moving forward.
    g_enc_r = m1;
    g_enc_l = m2;
    return 0;
}

/* ── init / cleanup ─────────────────────────────────────────────── */

int motor_hal_init(const char *device, int baud)
{
    if (!device)
        device = "/dev/ttyO2";
    if (baud <= 0)
        baud = 460800;

    LOG_INFO("motor_hal_roboclaw: opening %s at %d baud (addr 0x%02X)",
             device, baud, RC_ADDRESS);

    g_rc = roboclaw_init(device, baud);
    if (!g_rc)
    {
        LOG_ERROR("motor_hal_roboclaw: roboclaw_init() failed — check device and baud");
        return -1;
    }

    motor_hal_set_both(0.0f, 0.0f);

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
    motor_hal_set_both(0.0f, 0.0f);
    roboclaw_close(g_rc);
    g_rc = NULL;
    LOG_INFO("motor_hal_roboclaw: cleaned up");
}

/* ── motor output ───────────────────────────────────────────────── */

int motor_hal_set_both(float left, float right)
{
    if (!g_rc)
        return -1;

    /* clamp to ±1.0 */
    if (left  >  1.0f) left  =  1.0f;
    if (left  < -1.0f) left  = -1.0f;
    if (right >  1.0f) right =  1.0f;
    if (right < -1.0f) right = -1.0f;

    const float pol_l    = g_motor_config.pol_l;
    const float pol_r    = g_motor_config.pol_r;
    const int   mode     = g_motor_config.mode;
    const int   qpps_max = g_motor_config.qpps_max;
    const int   accel    = g_motor_config.accel_qpps;

    int ret;
    pthread_mutex_lock(&g_rc_mutex);

    if (mode == MOTOR_HAL_MODE_VELOCITY_ACCEL) {
        /* closed-loop velocity + acceleration ramp (MIXEDSPEEDACCEL, cmd 40) */
        int32_t spd_r = (int32_t)(pol_r * right * (float)qpps_max);
        int32_t spd_l = (int32_t)(pol_l * left  * (float)qpps_max);
        ret = roboclaw_speed_accel_m1m2(g_rc, RC_ADDRESS, spd_r, spd_l, accel);
        if (ret != ROBOCLAW_OK)
            LOG_WARN("motor_hal_roboclaw: speed_accel command failed (%d)", ret);

    } else if (mode == MOTOR_HAL_MODE_VELOCITY) {
        /* closed-loop velocity, no ramp (MIXEDSPEED, cmd 37) */
        int32_t spd_r = (int32_t)(pol_r * right * (float)qpps_max);
        int32_t spd_l = (int32_t)(pol_l * left  * (float)qpps_max);
        ret = roboclaw_speed_m1m2(g_rc, RC_ADDRESS, spd_r, spd_l);
        if (ret != ROBOCLAW_OK)
            LOG_WARN("motor_hal_roboclaw: speed command failed (%d)", ret);

    } else {
        /* raw PWM duty cycle (MIXEDDUTY, cmd 34) — default, no encoder feedback */
        int16_t d1 = (int16_t)(pol_r * right * (float)DUTY_MAX);
        int16_t d2 = (int16_t)(pol_l * left  * (float)DUTY_MAX);
        ret = roboclaw_duty_m1m2(g_rc, RC_ADDRESS, d1, d2);
        if (ret != ROBOCLAW_OK)
            LOG_WARN("motor_hal_roboclaw: duty command failed (%d)", ret);
    }

    pthread_mutex_unlock(&g_rc_mutex);
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
    return motor_hal_set_both(0.0f, 0.0f);
}

int motor_hal_standby(int standby)
{
    /* RoboClaw has no standby pin — zeroing motors is the equivalent */
    if (standby)
        return motor_hal_set_both(0.0f, 0.0f);
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
    if (!g_rc)
        return -1;
    pthread_mutex_lock(&g_rc_mutex);
    int ret = roboclaw_reset_encoders(g_rc, RC_ADDRESS);
    pthread_mutex_unlock(&g_rc_mutex);
    if (ret != ROBOCLAW_OK)
        LOG_WARN("motor_hal_roboclaw: hardware encoder reset failed (%d)", ret);
    return (ret == ROBOCLAW_OK) ? 0 : -1;
}