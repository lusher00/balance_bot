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
    pid->integrator_max = 1.0f;  // Default anti-windup limit
    pid->last_p_term = 0.0f;
    pid->last_i_term = 0.0f;
    pid->last_d_term = 0.0f;
    pid->last_output = 0.0f;
}

float pid_update(pid_controller_t* pid, float setpoint, float measurement) {
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
    
    // Derivative term
    float d_term = pid->kd * (error - pid->prev_error) / pid->dt;

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

void pid_reset(pid_controller_t* pid) {
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
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

