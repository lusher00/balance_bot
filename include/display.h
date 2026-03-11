/**
 * @file display.h
 * @brief ncurses live-display thread for balance_bot
 *
 * Runs in a background thread at ~10 Hz.  robot_run() just calls
 * display_update() each loop iteration to push fresh data; the thread
 * handles all terminal I/O so the control loop is never blocked.
 *
 * Blocks shown (each toggled by the -d flag in main.c):
 *
 *   sbus   SBUS TX channel values, switch states, failsafe
 *   pid    PID setpoint / error / P / I / D / output
 *   enc    Encoder ticks, position, velocity
 *   imu    Pitch / roll / yaw + rates
 *   mot    Motor duty cycles
 *   sys    Armed state, mode, battery, theta  (on by default)
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "debug_config.h"
#include "balance_bot.h"   /* robot_state_t, telemetry_data_t */

/**
 * @brief Start the ncurses display thread.
 *
 * Must be called after g_debug_config is fully initialised.
 * Has no effect if all display blocks are disabled.
 *
 * @return  0 on success, -1 on error
 */
int display_init(void);

/**
 * @brief Signal the display thread to redraw on its next wake.
 *
 * Call this once per robot_run() iteration.  Non-blocking.
 */
void display_update(void);

/**
 * @brief Stop the display thread and restore the terminal.
 *
 * Safe to call even if display_init() was never called.
 */
void display_cleanup(void);

#endif /* DISPLAY_H */
