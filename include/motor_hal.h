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

/**
 * @brief Reset RoboClaw after a latched e-stop.
 *
 * Closes the serial connection, runs roboclaw_reset.py (WriteNVM cmd 94),
 * then reinitialises.  Blocks ~2 s while the unit comes back up.
 * Only meaningful for the RoboClaw backend; the rc_motor backend is a no-op.
 *
 * @return 0 on success, -1 if reinit failed
 */
int motor_hal_roboclaw_reset(void);

/**
 * @brief Upload velocity PID gains to the RoboClaw hardware (M1 + M2).
 *
 * Only meaningful in velocity modes (mode 1 / 2).  Also updates
 * g_motor_config.claw_kp/ki/kd so the values survive a save_pid.
 * The rc_motor backend is a no-op (returns 0).
 *
 * @param kp  Proportional gain (float; encoded as kp × 65536 on the wire)
 * @param ki  Integral gain
 * @param kd  Derivative gain
 * @return 0 on success, -1 on error
 */
int motor_hal_set_claw_pid(float kp, float ki, float kd);

/**
 * @brief Read the RoboClaw main battery voltage.
 *
 * Queries the RoboClaw via GETMBATT (cmd 24).  The result is in volts
 * (e.g. 11.8 for a 3S LiPo at nominal charge).
 *
 * The rc_motor backend always sets *volts = 0.0f and returns -1.
 *
 * @param volts  Out: battery voltage in volts, or 0.0 on error
 * @return 0 on success, -1 on error
 */
int motor_hal_read_voltage(float *volts);

/**
 * @brief Read encoder speeds directly from RoboClaw hardware (QPPS).
 *
 * Uses GETM1SPEED/GETM2SPEED (cmds 18/19).  The RoboClaw measures
 * inter-pulse timing internally so this returns non-zero even at slow
 * speeds where tick-delta methods return 0.
 *
 * Combined signed QPPS sum (m1 + m2) / 10 approximates ticks/100ms.
 *
 * @param m1_qpps  Out: M1 signed speed in pulses/second
 * @param m2_qpps  Out: M2 signed speed in pulses/second
 * @return 0 on success, -1 on error
 */
int motor_hal_read_encoder_speeds(int32_t *m1_qpps, int32_t *m2_qpps);

/**
 * @brief Read RoboClaw board temperature.
 *
 * @param temp_c  Out: temperature in °C, or 0.0 on error
 * @return 0 on success, -1 on error
 */
int motor_hal_read_temp(float *temp_c);

/**
 * @brief Change the serial baud rate for the RoboClaw connection on the fly.
 *
 * Closes the current serial connection and reopens at the new baud rate.
 * Also updates g_motor_config.baud so the value persists through save_pid.
 * Use this when the RoboClaw baud has been changed and the BBB needs to
 * match.  Blocks briefly while the port is recycled (~50 ms).
 *
 * @param baud  New baud rate (e.g. 115200, 460800)
 * @return 0 on success, -1 on error
 */
int motor_hal_set_baud(int baud);

#endif /* MOTOR_HAL_H */