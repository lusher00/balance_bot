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
 * @file motor_hal.h
 * @brief Motor and encoder hardware abstraction layer.
 *
 * All motor and encoder calls in robot.c go through this interface.
 * To swap hardware, replace the corresponding motor_hal_*.c source
 * in the build — nothing else changes.
 *
 * Two backends are provided:
 *
 *   motor_hal_rc.c     — BeagleBone rc_motor PWM cape + rc_encoder_eqep
 *                        (default, no extra wiring needed)
 *
 *   motor_hal_roboclaw.c — RoboClaw packet-serial over a UART
 *                          (pass -m /dev/ttyOx to main)
 *
 * Adding a new backend: implement every function below in a new
 * motor_hal_<name>.c and swap it into SRCS in the Makefile.
 */

#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>

/* ── channel indices ────────────────────────────────────────────────
 * Use these everywhere instead of raw channel numbers so a backend
 * can remap them internally.
 */
#define MOTOR_RIGHT 0
#define MOTOR_LEFT  1

/* ── init / cleanup ─────────────────────────────────────────────── */

/**
 * @brief Initialise the motor driver and encoder hardware.
 *
 * For the rc_motor backend:   calls rc_motor_init() + rc_encoder_eqep_init()
 * For the RoboClaw backend:   opens the UART, sends a zero-speed packet
 *
 * @param device  UART device path for backends that need it (e.g. "/dev/ttyO2").
 *                Pass NULL for the rc_motor backend — it is ignored.
 * @param baud    Baud rate for UART backends.  Ignored by rc_motor backend.
 * @return 0 on success, -1 on error
 */
int  motor_hal_init   (const char *device, int baud);

/** @brief Stop motors, release hardware, close any open UART. */
void motor_hal_cleanup(void);

/* ── motor output ───────────────────────────────────────────────── */

/**
 * @brief Set a single motor duty cycle.
 *
 * @param motor  MOTOR_LEFT or MOTOR_RIGHT
 * @param duty   -1.0 (full reverse) .. +1.0 (full forward)
 * @return 0 on success, -1 on error
 */
int motor_hal_set(int motor, float duty);

/**
 * @brief Set both motors in one call (preferred — avoids split-packet lag).
 *
 * @param left   duty for MOTOR_LEFT
 * @param right  duty for MOTOR_RIGHT
 * @return 0 on success, -1 on error
 */
int motor_hal_set_both(float left, float right);

/**
 * @brief Coast both motors (remove drive, let them spin freely).
 * Distinct from set(0) which may actively brake depending on the driver.
 */
int motor_hal_free_spin(void);

/**
 * @brief Put motors into low-power standby (braking).
 * @param standby 1 = engage standby, 0 = exit standby
 */
int motor_hal_standby(int standby);

/* ── encoder input ──────────────────────────────────────────────── */

/**
 * @brief Read raw encoder tick count (signed, accumulating).
 *
 * @param motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return Signed tick count, or 0 on error
 */
int32_t motor_hal_encoder_read(int motor);

/**
 * @brief Zero the encoder count for one motor.
 *
 * @param motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return 0 on success, -1 on error
 */
int motor_hal_encoder_reset(int motor);

/**
 * @brief Zero both encoder counts.
 * @return 0 on success, -1 on error
 */
int motor_hal_encoder_reset_all(void);
int motor_hal_roboclaw_reset(void);

#endif /* MOTOR_HAL_H */
