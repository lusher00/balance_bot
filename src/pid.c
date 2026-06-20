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
    
    // Return total output
    return p_term + i_term + d_term;
}

void pid_reset(pid_controller_t* pid) {
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set_gains(pid_controller_t* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

