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
 * @file motor_hal_rc.c
 * @brief Motor HAL backend: BeagleBone rc_motor PWM cape + rc_encoder_eqep.
 *
 * This is the default backend.  It wraps the librobotcontrol functions
 * directly.  Motor channel numbers and polarities come from the defines
 * in rc_balance_defs.h (or balance_bot.h) — edit those to match your wiring.
 *
 * To use a different backend, swap this file for motor_hal_roboclaw.c
 * (or your own motor_hal_*.c) in the Makefile SRCS list.
 */

#include "motor_hal.h"
#include "balance_bot.h"   /* LOG_*, rc_motor, rc_encoder_eqep */

#include <robotcontrol.h>

/* ── wiring — edit to match your cape ──────────────────────────── */
#define CH_L        3           /* rc_motor / rc_encoder_eqep channel for left  */
#define CH_R        2           /* rc_motor / rc_encoder_eqep channel for right */
#define POL_L      ( 1.0f)      /* flip if left  motor runs backwards            */
#define POL_R      ( 1.0f)      /* flip if right motor runs backwards            */
#define ENC_POL_L  ( 1)         /* flip if left  encoder counts backwards        */
#define ENC_POL_R  ( 1)         /* flip if right encoder counts backwards        */

/* ── channel lookup ─────────────────────────────────────────────── */
static inline int ch(int motor)   { return motor == MOTOR_LEFT ? CH_L  : CH_R;  }
static inline float pol(int motor){ return motor == MOTOR_LEFT ? POL_L : POL_R; }
static inline int epol(int motor) { return motor == MOTOR_LEFT ? ENC_POL_L : ENC_POL_R; }

/* ── init / cleanup ─────────────────────────────────────────────── */

int motor_hal_init(const char *device, int baud)
{
    (void)device;   /* not used by this backend */
    (void)baud;

    if (rc_encoder_eqep_init() < 0) {
        LOG_ERROR("motor_hal_rc: rc_encoder_eqep_init() failed");
        return -1;
    }
    if (rc_motor_init() < 0) {
        LOG_ERROR("motor_hal_rc: rc_motor_init() failed");
        rc_encoder_eqep_cleanup();
        return -1;
    }
    rc_motor_standby(1);    /* start with motors off */

    LOG_INFO("motor_hal_rc: ready (L=ch%d pol%+.0f, R=ch%d pol%+.0f)",
             CH_L, POL_L, CH_R, POL_R);
    return 0;
}

void motor_hal_cleanup(void)
{
    rc_motor_set(0, 0.0);       /* channel 0 = all motors */
    rc_motor_standby(1);
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
    LOG_INFO("motor_hal_rc: cleaned up");
}

/* ── motor output ───────────────────────────────────────────────── */

int motor_hal_set(int motor, float duty)
{
    return rc_motor_set(ch(motor), pol(motor) * duty);
}

int motor_hal_set_both(float left, float right)
{
    int r = rc_motor_set(CH_L, POL_L * left);
    r    |= rc_motor_set(CH_R, POL_R * right);
    return r ? -1 : 0;
}

int motor_hal_free_spin(void)
{
    return rc_motor_free_spin(0);   /* 0 = all channels */
}

int motor_hal_standby(int standby)
{
    return rc_motor_standby(standby);
}

/* ── encoder input ──────────────────────────────────────────────── */

int32_t motor_hal_encoder_read(int motor)
{
    return (int32_t)(rc_encoder_eqep_read(ch(motor)) * epol(motor));
}

int motor_hal_encoder_reset(int motor)
{
    return rc_encoder_eqep_write(ch(motor), 0);
}

int motor_hal_encoder_reset_all(void)
{
    int r  = rc_encoder_eqep_write(CH_L, 0);
    r     |= rc_encoder_eqep_write(CH_R, 0);
    return r ? -1 : 0;
}

/* ── RoboClaw velocity PID (stub — rc_motor backend has no velocity PID) ── */

int motor_hal_set_claw_pid(float kp, float ki, float kd)
{
    (void)kp; (void)ki; (void)kd;
    return 0;
}

int motor_hal_read_voltage(float *volts)
{
    *volts = 0.0f;
    return -1;
}

int motor_hal_read_encoder_speeds(int32_t *m1_qpps, int32_t *m2_qpps)
{
    *m1_qpps = 0;
    *m2_qpps = 0;
    return -1;
}

int motor_hal_read_temp(float *temp_c)
{
    *temp_c = 0.0f;
    return -1;
}

int motor_hal_set_baud(int baud)
{
    (void)baud;
    return 0;
}
