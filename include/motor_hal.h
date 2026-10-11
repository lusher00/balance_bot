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

/* Motion gate. motor_hal_set_both() is the ONLY function in this program that
 * sends a non-zero drive command to the RoboClaw. Before every such command it
 * asks this gate; if the gate says no (or none is registered yet) it sends a
 * COAST (duty 0) instead. Default is deny: from process start until robot.c
 * registers the gate AND its conditions hold, nothing can drive the wheels.
 * The gate returns 1 to permit; on deny it sets *why to a short reason. */
typedef int (*motor_hal_gate_fn)(const char **why);
void motor_hal_set_motion_gate(motor_hal_gate_fn fn);

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

/* A REAL stop: raw duty zero (MIXEDDUTY, cmd 34), whatever mode is configured.
 *
 * This exists because motor_hal_set_both(0,0) is not one. In velocity mode it
 * sends MIXEDSPEED with a target of zero, which hands the RoboClaw's own
 * velocity PID (kp=15 ki=2 kd=0.5) the job of HOLDING zero -- an active
 * closed loop that drives the motors to make the encoders read what it wants,
 * winds its integral up against stiction, overshoots, and reverses. On the
 * bench that is a robot that starts pushing the moment the service comes up
 * and never stops, which is exactly what it did on 2026-09-13.
 *
 * Duty zero removes power instead of regulating to zero. Use this anywhere the
 * intent is "motors off": startup, the OOB cutoff, disarm, e-stop. Use
 * motor_hal_set_both() only when the intent is "drive at this value". */
int motor_hal_coast(void);

/**
 * @brief Drive both wheels at a raw PWM duty, ignoring the configured mode.
 *
 * For the stand-up kick, and nothing else. In velocity mode set_both() turns
 * its argument into a SPEED target — 0.35 with qpps_max 500 asks for 175
 * QPPS, a crawl — and how much torque the controller then applies is up to
 * its velocity PID, which is tuned for balancing a robot that is already
 * upright. A kick needs the opposite: all the voltage there is, for a bounded
 * moment, with the current limit as the only ceiling.
 *
 * Goes through the motion gate like every other motion command.
 *
 * @param duty -1.0..1.0, both wheels the same
 * @return 0 if the command reached the controller, -1 otherwise
 */
int motor_hal_kick(float duty);

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
 * @brief Read both motor currents in amps (signed; negative = regen/braking).
 * @return 0 on success, -1 if unavailable.
 */
int motor_hal_read_currents(float *m1_amps, float *m2_amps);

/**
 * @brief Set the per-motor hardware current limit on the controller.
 * @param amps per-motor limit; <= 0 leaves the controller's setting untouched.
 */
int motor_hal_set_current_limit(float amps);

/** @brief Read back the per-motor current limit the controller is enforcing. */
int motor_hal_read_current_limit(float *amps);

/* What the RoboClaw itself reports, read back over serial -- NOT balance_bot's
 * copy of robot.conf.  If the controller was configured in Ion Studio (motor
 * or encoder inverted inside the unit), robot.conf cannot tell you and this
 * can.  Settings, not signals: poll it rarely, it costs six serial round
 * trips.  Returns 0 on success, -1 if any read failed. */
typedef struct
{
    float m1_kp, m1_ki, m1_kd;
    uint32_t m1_qpps;
    float m2_kp, m2_ki, m2_kd;
    uint32_t m2_qpps;
    uint8_t enc_mode_m1;   /* GETENCODERMODE, raw */
    uint8_t enc_mode_m2;
    uint16_t config;       /* GETCONFIG, raw */
    /* Health and limits, for the dashboard's Claw tab. */
    float main_v, logic_v; /* GETMBATT / GETLBATT */
    float temp_c;          /* GETTEMP */
    uint32_t status;       /* GETERROR: error + warning bits */
    float batt_min, batt_max;       /* GETMINMAXMAINVOLTAGES */
    float max_amps_m1, max_amps_m2; /* GETM1/M2MAXCURRENT */
    uint8_t pins[5];       /* GETPINFUNCTIONS: S3 S4 S5 CTRL1 CTRL2 */
    int valid;   /* int, not bool: motor_hal.h does not pull in stdbool */
} claw_hw_settings_t;

int motor_hal_read_hw_settings(claw_hw_settings_t *out);

/* Main battery limits on the RoboClaw (cmd 57). vmin <= 0 leaves the
 * controller alone; vmax <= 0 keeps the controller's current maximum. RAM only
 * on firmware v4.2.8 -- WriteNVM resets it -- so it is re-applied at every
 * start and after every e-stop reset. */
int motor_hal_set_batt_limits(float vmin, float vmax);

/* Push everything robot.conf owns back into the RoboClaw: current limit,
 * velocity PID, battery limits. Used after WriteNVM (CLR ESTOP), which on this
 * firmware resets the controller's configuration. */
void motor_hal_apply_claw_settings(void);

/* Non-zero when the RoboClaw has stopped answering and serial I/O is being
 * skipped to protect the control loop rate. Clears itself when it answers
 * again -- no restart needed. */
int motor_hal_link_down(void);

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

/* ── Wire-level diagnostics for the 100 Hz ring ─────────────────────────
 * What actually went to and came back from the RoboClaw, recorded as a side
 * effect of calls the control loop already makes. Filling this costs no serial
 * I/O. The two poll functions below are the only extra traffic, and robot.c
 * runs each of them once every 10 control ticks on separate ticks.
 *
 * Wheel convention throughout: *_l = MOTOR_LEFT (RoboClaw M2), *_r =
 * MOTOR_RIGHT (M1). Wire values are after pol_l/pol_r, speeds after
 * enc_pol_l/enc_pol_r, i.e. the same sign convention as enc_left/enc_right. */
enum
{
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_SPEED = 1,       /* MIXEDSPEED (37), wire units QPPS          */
    MOTOR_CMD_SPEED_ACCEL = 2, /* MIXEDSPEEDACCEL (40), wire units QPPS     */
    MOTOR_CMD_DUTY = 3,        /* MIXEDDUTY (34) from set_both, +/-32767    */
    MOTOR_CMD_COAST = 4,       /* MIXEDDUTY 0                               */
    MOTOR_CMD_KICK = 5,        /* MIXEDDUTY from motor_hal_kick             */
    MOTOR_CMD_LINK_SKIP = 6,   /* nothing sent: link marked down            */
};

typedef struct
{
    /* last motor command */
    uint32_t cmd_seq;    /* +1 per command attempt; unchanged = nothing sent */
    int32_t cmd_kind;    /* MOTOR_CMD_* */
    int32_t cmd_wire_l;  /* value sent for the left motor (M2) */
    int32_t cmd_wire_r;  /* value sent for the right motor (M1) */
    int32_t cmd_ret;     /* roboclaw return code, 0 = OK */
    uint32_t cmd_us;     /* serial round trip incl. mutex wait */
    uint32_t cmd_fails;  /* cumulative */
    int32_t gate_ok;     /* motion gate on the last drive request */
    /* encoder position read (every tick) */
    int32_t enc_ret;
    uint32_t enc_us;
    uint32_t enc_fails;
    /* RoboClaw's own speed measurement (GETISPEEDS, polled by robot.c) */
    int32_t spd_l, spd_r;
    uint64_t spd_t_us;
    /* currents (telemetry.c, 20 Hz) */
    float amps_l, amps_r;
    uint64_t amps_t_us;
    /* main battery at the RoboClaw */
    float vbatt;
    uint64_t vbatt_t_us;
    /* GETERROR status word */
    uint32_t status;
    int32_t status_ret;
    uint64_t status_t_us;
} motor_hal_diag_t;

/** Copy of the current diagnostics. No I/O. */
void motor_hal_get_diag(motor_hal_diag_t *out);

/** Read the RoboClaw status/error word (GETERROR, cmd 90) into the diag
 *  struct. One serial round trip. Does not touch the link watchdog. */
int motor_hal_poll_status(void);

/** Read main battery voltage (GETMBATT, cmd 24) into the diag struct. One
 *  serial round trip. Does not touch the link watchdog. */
int motor_hal_poll_vbatt(void);

#endif /* MOTOR_HAL_H */