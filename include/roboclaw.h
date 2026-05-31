/*
 * roboclaw library header
 * 
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */

#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> //uint8_t, int16_t

//internal library data
struct roboclaw;

//error codes returned by the functions
enum {ROBOCLAW_ERROR=-1, ROBOCLAW_RETRIES_EXCEEDED=-2, ROBOCLAW_OK=0};

/* Initialize roboclaw communication with default values
 * 
 * NULL is returned most likely on incorrect tty device or permission error (are you in dialout group?)
 * 
 * parameters:
 * - tty - the device (e.g. /dev/ttyAMA0, /dev/ttyUSB0, /dev/ttyO1)
 * - baudrate - has to match the one set on roboclaw (2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800)
 * 
 * returns:
 * NULL on failure, and pointer to library data on success, on failure you can query errno for details about error
 *  
*/
struct roboclaw *roboclaw_init(const char *tty, int baudrate);


/* Initialize roboclaw communication
 * 
 * NULL is returned most likely on incorrect tty device or permission error (are you in dialout group?)
 * 
 * parameters:
 * - tty - the device (e.g. /dev/ttyAMA0, /dev/ttyUSB0, /dev/ttyO1)
 * - baudrate - has to match the one set on roboclaw (2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800)
 * - retries - the number of retries the library should make before reporting failure (both timeout and incorrect crc)
 * - timeout_ms - timeout in ms for the roundtrip sending command and waiting for ACK or response
 * - strict_0xFF_ACK - require strict 0xFF ACK byte matching (treat non 0xFF as crc failure
 * 
 * returns:
 * NULL on failure, and pointer to library data on success, on failure you can query errno for details about error
 *  
*/
struct roboclaw *roboclaw_init_ext(const char *tty, int baudrate, int timeout_ms, int retries, int strict_0xFF_ACK);


/*
 * Cleans after library 
 *
 * Closes file descriptor, resets terminal to initial settings and frees the memory.
 * Calling this function with NULL argument is legal and will result in ROBOCLAW_OK
 * 
 * parameters:
 * rc - pointer to library internal data
 * 
 * returns:
 * - ROBOCLAW_OK on success, ROBOCLAW_FAILURE on failure and errno is set
 */
int roboclaw_close(struct roboclaw *rc);


/*
 * All the commands return:
 * - ROBOCLAW_OK on success
 * - ROBOCLAW_ERROR on IO error with errno set
 * - ROBOCLAW_RETRIES_EXCEEDED if sending didn't result in reply or the checksum didn't match `replies` times (configurable)
 * 
 * The failure codes are guarateed to have negative values
 * so in simplest scenario it is enough to test for ROBOCLAW_OK
 * 
 * All the commands take parametrs:
 * - rc - pointer to library internal data
 * - address - the address of roboclaw that should receive command (from 0x80 to 0x87)
 * - ...
 * - and command specific parameters
 * 
 */

int roboclaw_duty_m1m2(struct roboclaw *rc, uint8_t address, int16_t duty_m1, int16_t duty_m2);
int roboclaw_speed_m1m2(struct roboclaw *rc,uint8_t address, int speed_m1, int speed_m2);
int roboclaw_speed_accel_m1m2(struct roboclaw *rc,uint8_t address, int speed_m1, int speed_m2, int accel);
int roboclaw_main_battery_voltage(struct roboclaw *rc,uint8_t address, int16_t *voltage);
int roboclaw_encoders(struct roboclaw *rc, uint8_t address, int32_t *enc_m1, int32_t *enc_m2);
int roboclaw_reset_encoders(struct roboclaw *rc, uint8_t address);

/**
 * @brief RoboClaw internal velocity PID parameters.
 *
 * The RoboClaw stores these as fixed-point uint32 (value × 65536).
 * Pass plain floats here; the wire encoding is handled internally.
 * Applies to both motors simultaneously (M1 cmd 28, M2 cmd 29).
 *
 * Default factory values: Kp=1.0  Ki=0.5  Kd=0.25
 * Increase Kp for stiffer speed response; add Ki to eliminate steady-state
 * speed error; Kd damps oscillation.  Always re-tune after changing qpps.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
} roboclaw_vel_pid_t;

/**
 * @brief Upload velocity PID gains to both M1 and M2.
 *
 * @param rc      roboclaw handle
 * @param address device address (e.g. 0x01)
 * @param pid     pointer to kp/ki/kd values
 * @param qpps    max encoder speed (pulses/sec) at full throttle —
 *                should match the qpps_max already in motor_config
 * @return ROBOCLAW_OK on success, ROBOCLAW_ERROR / ROBOCLAW_RETRIES_EXCEEDED on failure
 */
int roboclaw_set_velocity_pid(struct roboclaw *rc, uint8_t address,
                              const roboclaw_vel_pid_t *pid, uint32_t qpps);

/**
 * @brief Read instantaneous encoder speed from RoboClaw hardware.
 *
 * Uses GETM1SPEED (cmd 18) and GETM2SPEED (cmd 19).  The RoboClaw measures
 * the time between encoder pulses internally and returns pulses/second —
 * non-zero even at very low speeds where tick-delta methods return 0.
 *
 * @param rc       roboclaw handle
 * @param address  device address
 * @param m1_qpps  Out: M1 speed in signed QPPS (positive = forward)
 * @param m2_qpps  Out: M2 speed in signed QPPS
 * @return ROBOCLAW_OK on success
 */
int roboclaw_encoder_speeds(struct roboclaw *rc, uint8_t address,
                            int32_t *m1_qpps, int32_t *m2_qpps);

/**
 * @brief Read RoboClaw board temperature (cmd 82).
 *
 * @param rc       roboclaw handle
 * @param address  device address
 * @param temp_c   Out: temperature in °C (value × 10 on wire; e.g. 324 = 32.4°C)
 * @return ROBOCLAW_OK on success
 */
int roboclaw_temperature(struct roboclaw *rc, uint8_t address, float *temp_c);

#ifdef __cplusplus
}
#endif

#endif // ROBOCLAW_H
