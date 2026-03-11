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

