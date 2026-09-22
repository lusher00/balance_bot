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
 * @file pid.c
 * @brief PID controller implementation
 */

#include "balance_bot.h"
#include <string.h>

void pid_init(pid_controller_t* pid, float kp, float ki, float kd, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->primed = 0;
    pid->integrator_max = 1.0f;  // Default anti-windup limit
    pid->last_p_term = 0.0f;
    pid->last_i_term = 0.0f;
    pid->last_d_term = 0.0f;
    pid->last_output = 0.0f;
}

/* Core. `rate` is d(measurement)/dt; pass NAN to have it differenced from the
 * stored previous measurement (the ordinary case). */
static float pid_core(pid_controller_t* pid, float setpoint, float measurement,
                      float rate, int have_rate) {
    float error = setpoint - measurement;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with anti-windup
    pid->integrator += error * pid->dt;
    if (pid->integrator > pid->integrator_max) {
        pid->integrator = pid->integrator_max;
    } else if (pid->integrator < -pid->integrator_max) {
        pid->integrator = -pid->integrator_max;
    }
    float i_term = pid->ki * pid->integrator;
    
    // Derivative term -- on the MEASUREMENT, not the error.
    //
    // d(error)/dt = d(setpoint)/dt - d(measurement)/dt. The pitch setpoint is
    // theta_ref from the position loop, and its velocity-damping part changes
    // in steps each time the RoboClaw speed reading updates (every ~5 ticks).
    // Differentiating those steps kicked the motors: in looplogs 7/12/16 the
    // setpoint part of d_term had a larger sd (0.068-0.076) than the part from
    // actual body motion (0.043-0.055), with single-tick kicks up to 0.81 duty
    // while balancing needs ~0.1. Those kicks are the random "jumps".
    //
    // -kd * d(measurement)/dt is the same damping on real body motion, and a
    // setpoint step now reaches the motors only through kp (smoothly), never
    // as a one-tick spike. Standard kick-free PID form.
    //
    // First update after init/reset: no history, so no derivative that tick.
    if (!pid->primed) {
        pid->prev_measurement = measurement;
        pid->primed = 1;
    }
    /* With a measured rate there is nothing to prime and no staircase to
     * differentiate: use it directly. prev_measurement is still tracked so a
     * switch back to the differenced form mid-run does not see a stale gap. */
    float d_term = have_rate
                       ? -pid->kd * rate
                       : -pid->kd * (measurement - pid->prev_measurement) / pid->dt;
    pid->prev_measurement = measurement;

    // Save error for next iteration
    pid->prev_error = error;

    float output = p_term + i_term + d_term;

    // Record the actual terms for telemetry. d_term in particular cannot be
    // recovered later because prev_error has just been overwritten.
    pid->last_p_term = p_term;
    pid->last_i_term = i_term;
    pid->last_d_term = d_term;
    pid->last_output = output;

    return output;
}

float pid_update(pid_controller_t* pid, float setpoint, float measurement) {
    return pid_core(pid, setpoint, measurement, 0.0f, 0);
}

float pid_update_rate(pid_controller_t* pid, float setpoint, float measurement,
                      float rate) {
    return pid_core(pid, setpoint, measurement, rate, 1);
}

void pid_reset(pid_controller_t* pid) {
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->primed = 0;
    pid->last_p_term = 0.0f;
    pid->last_i_term = 0.0f;
    pid->last_d_term = 0.0f;
    pid->last_output = 0.0f;
}

void pid_set_gains(pid_controller_t* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

