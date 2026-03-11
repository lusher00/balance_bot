/**
 * @file motor_hal_roboclaw.c
 * @brief Motor HAL backend: RoboClaw via roboclaw library (Bartosz Meglicki).
 *
 * Uses roboclaw_init() / roboclaw_duty_m1m2() / roboclaw_encoders() from
 * the vendored roboclaw library (src/roboclaw.c + include/roboclaw.h).
 * That library owns the tty directly via termios — do NOT also open it
 * through rc_uart.
 *
 * To activate: in the Makefile set
 *   MOTOR_HAL = src/motor_hal_roboclaw.c
 * and pass -m /dev/ttyOx [-B <baud>] on the command line.
 *
 * Wiring notes:
 *   M1 = LEFT  motor  (adjust POL_L if it runs backwards)
 *   M2 = RIGHT motor  (adjust POL_R if it runs backwards)
 *   Address 0x80 is the RoboClaw factory default.
 */

#include "motor_hal.h"
#include "balance_bot.h"
#include "roboclaw.h"

#include <string.h>
#include <stdint.h>

/* ── wiring ─────────────────────────────────────────────────────── */
#define RC_ADDRESS  0x80    /* default RoboClaw address — change if you set a different one */
#define POL_L       1.0f    /* flip to -1.0f if left  motor runs backwards */
#define POL_R      -1.0f    /* flip to  1.0f if right motor runs backwards */

/* duty range the RoboClaw expects for MIXEDDUTY (cmd 34): -32767 .. +32767 */
#define DUTY_MAX    32767

static struct roboclaw *g_rc = NULL;

/* last encoder values — cached so motor_hal_encoder_read() works per-side */
static int32_t g_enc_l = 0;
static int32_t g_enc_r = 0;

/* ── helpers ────────────────────────────────────────────────────── */

static int refresh_encoders(void)
{
    if (!g_rc) return -1;
    int ret = roboclaw_encoders(g_rc, RC_ADDRESS, &g_enc_l, &g_enc_r);
    if (ret != ROBOCLAW_OK) {
        LOG_WARN("motor_hal_roboclaw: encoder read failed (%d)", ret);
        return -1;
    }
    return 0;
}

/* ── init / cleanup ─────────────────────────────────────────────── */

int motor_hal_init(const char *device, int baud)
{
    if (!device) device = "/dev/ttyO2";
    if (baud <= 0) baud = 38400;

    LOG_INFO("motor_hal_roboclaw: opening %s at %d baud (addr 0x%02X)",
             device, baud, RC_ADDRESS);

    g_rc = roboclaw_init(device, baud);
    if (!g_rc) {
        LOG_ERROR("motor_hal_roboclaw: roboclaw_init() failed — check device and baud");
        return -1;
    }

    motor_hal_set_both(0.0f, 0.0f);
    LOG_INFO("motor_hal_roboclaw: ready");
    return 0;
}

void motor_hal_cleanup(void)
{
    if (!g_rc) return;
    motor_hal_set_both(0.0f, 0.0f);
    roboclaw_close(g_rc);
    g_rc = NULL;
    LOG_INFO("motor_hal_roboclaw: cleaned up");
}

/* ── motor output ───────────────────────────────────────────────── */

int motor_hal_set_both(float left, float right)
{
    if (!g_rc) return -1;

    /* clamp */
    if (left  >  1.0f) left  =  1.0f; if (left  < -1.0f) left  = -1.0f;
    if (right >  1.0f) right =  1.0f; if (right < -1.0f) right = -1.0f;

    int16_t d1 = (int16_t)(POL_L * left  * DUTY_MAX);
    int16_t d2 = (int16_t)(POL_R * right * DUTY_MAX);

    int ret = roboclaw_duty_m1m2(g_rc, RC_ADDRESS, d1, d2);
    if (ret != ROBOCLAW_OK) {
        LOG_WARN("motor_hal_roboclaw: duty command failed (%d)", ret);
        return -1;
    }
    return 0;
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
    if (standby) return motor_hal_set_both(0.0f, 0.0f);
    return 0;
}

/* ── encoder input ──────────────────────────────────────────────── */

int32_t motor_hal_encoder_read(int motor)
{
    refresh_encoders();
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
    /* roboclaw library doesn't wrap cmd 20 (RESETENC) directly.
     * Use roboclaw_encoders() with a trick: set both encoder counts to 0
     * by sending SETM1ENCCOUNT / SETM2ENCCOUNT if available, otherwise
     * just zero our cached values and accept the hardware count continuing.
     *
     * Simplest correct approach: the library does expose roboclaw_encoders
     * for reading only.  For reset we fall back to zeroing the cache here
     * and note that robot.c should call this before arming so relative
     * position is what matters, not absolute.
     *
     * TODO: add roboclaw_reset_encoders() to the library if needed.
     */
    g_enc_l = 0;
    g_enc_r = 0;
    LOG_WARN("motor_hal_roboclaw: encoder reset zeroes cache only — "
             "hardware counters not reset (add RESETENC cmd to library if needed)");
    return 0;
}
