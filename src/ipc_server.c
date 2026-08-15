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
 * @file ipc_server.c
 * @brief Unix domain socket server for iPhone app communication
 *
 * This module provides a Unix socket server that allows the Node.js
 * web server to communicate with the balance_bot C application.
 *
 * Communication is bidirectional:
 * - C app → Node.js: Telemetry data (JSON)
 * - Node.js → C app: Commands from iPhone (JSON)
 *
 * Socket path: /tmp/balance_bot.sock
 */

#include "debug_config.h"
#include "balance_bot.h"
#include "motor_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#ifdef __linux__
#include <linux/sockios.h>   /* SIOCOUTQ — see client_has_headroom() */
#endif
#include "rc_compat.h"
#include "roboclaw_estop.h"

#define SOCKET_PATH "/tmp/balance_bot.sock"
#define MAX_CLIENTS 5
#define BUFFER_SIZE 4096

// Forward declarations
static void *client_handler_thread(void *arg);
static int handle_command(const char *json_cmd, char *response, size_t response_len);
static int parse_json_command(const char *json_cmd);
static void build_telemetry_json(char *buffer, size_t size);
static void build_rc_json(char *buffer, size_t size);
static void build_config_json(char *buffer, size_t size);

// Client connection tracking
typedef struct
{
    int socket_fd;
    bool active;
    pthread_t thread;
    /* Frames skipped because this client's socket buffer was full. Non-zero
     * here means the client is not keeping up, which is a property of the
     * client or the link -- not of the control loop. */
    unsigned long dropped;
    /* RC packets deliberately skipped to preserve telemetry headroom. Not a
     * fault -- see TX_RESERVE_BYTES. */
    unsigned long rc_yielded;
} client_connection_t;

static client_connection_t clients[MAX_CLIENTS] = {0};
static pthread_mutex_t clients_mutex = PTHREAD_MUTEX_INITIALIZER;

static int server_socket = -1;
static pthread_t server_thread;
static bool server_running = false;

/**
 * @brief Main server thread - accepts new connections
 */
static void *server_thread_func(void *arg __attribute__((unused)))
{
    int client_fd;

    LOG_INFO("IPC server thread started");

    while (server_running)
    {
        // Accept new connection
        client_fd = accept(server_socket, NULL, NULL);
        if (client_fd < 0)
        {
            if (errno == EINTR)
                continue;
            LOG_ERROR("Accept failed: %s", strerror(errno));
            break;
        }

        LOG_INFO("New client connected (fd=%d)", client_fd);

        /* Non-blocking from the moment we own it. ipc_broadcast_line() runs on
         * the control loop thread; on a blocking fd a client that stops draining
         * would stall the balance loop inside write(). O_NONBLOCK is a property
         * of the open file description, so it applies to the read side too --
         * client_handler_thread() therefore poll()s before reading rather than
         * spinning on EAGAIN. */
        int fl = fcntl(client_fd, F_GETFL, 0);
        if (fl < 0 || fcntl(client_fd, F_SETFL, fl | O_NONBLOCK) < 0)
        {
            LOG_ERROR("Failed to set O_NONBLOCK on fd=%d: %s",
                      client_fd, strerror(errno));
            close(client_fd);
            continue;
        }

        // Find free slot for client
        pthread_mutex_lock(&clients_mutex);
        bool added = false;
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            if (!clients[i].active)
            {
                clients[i].socket_fd = client_fd;
                clients[i].active = true;
                clients[i].dropped = 0;
                clients[i].rc_yielded = 0;

                // Spawn handler thread
                if (pthread_create(&clients[i].thread, NULL,
                                   client_handler_thread, &clients[i]) != 0)
                {
                    LOG_ERROR("Failed to create client handler thread");
                    close(client_fd);
                    clients[i].active = false;
                }
                else
                {
                    pthread_detach(clients[i].thread);
                    added = true;
                }
                break;
            }
        }
        pthread_mutex_unlock(&clients_mutex);

        /* A fresh client has no motor/pos/sbus config yet -- those are no longer
         * repeated in every telemetry packet, so without this its controls would
         * sit at their built-in defaults until someone happened to change a
         * setting. Flagging it here lets the CONTROL LOOP do the actual send, so
         * every write to a client fd still comes from one thread and a config
         * packet can never interleave into the middle of a telemetry packet. */
        if (added)
            ipc_config_touch();

        if (!added)
        {
            LOG_WARN("Max clients reached, rejecting connection");
            close(client_fd);
        }
    }

    LOG_INFO("IPC server thread stopped");
    return NULL;
}

/**
 * @brief Client handler thread - reads commands from one client
 */
static void *client_handler_thread(void *arg)
{
    client_connection_t *client = (client_connection_t *)arg;
    char buffer[BUFFER_SIZE];
    char response[BUFFER_SIZE];
    int bytes_read;

    LOG_DEBUG("Client handler started for fd=%d", client->socket_fd);

    while (client->active)
    {
        /* The fd is O_NONBLOCK (set in the accept path so the control loop can
         * never block writing to it), so a bare read() would spin on EAGAIN and
         * burn a core. Wait for readability instead. The timeout is what lets
         * this thread notice client->active going false at shutdown. */
        struct pollfd pfd = { .fd = client->socket_fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 200);
        if (pr == 0)
            continue;                 /* timeout -- re-check client->active */
        if (pr < 0)
        {
            if (errno == EINTR)
                continue;
            LOG_ERROR("poll failed on fd=%d: %s", client->socket_fd, strerror(errno));
            break;
        }
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))
            break;

        bytes_read = read(client->socket_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
            continue;

        if (bytes_read <= 0)
        {
            if (bytes_read < 0)
            {
                LOG_ERROR("Read error on fd=%d: %s", client->socket_fd, strerror(errno));
            }
            break;
        }

        buffer[bytes_read] = '\0';
        LOG_DEBUG("Received command: %s", buffer);

        // Process command
        if (handle_command(buffer, response, sizeof(response)) == 0)
        {
            // Send response
            ssize_t written = write(client->socket_fd, response, strlen(response));
            (void)written; // Suppress unused warning
        }
    }

    LOG_INFO("Client disconnected (fd=%d)", client->socket_fd);
    close(client->socket_fd);

    pthread_mutex_lock(&clients_mutex);
    client->active = false;
    pthread_mutex_unlock(&clients_mutex);

    return NULL;
}

/**
 * @brief Handle incoming JSON command from iPhone
 *
 * @param json_cmd Null-terminated JSON command string
 * @param response Buffer to write JSON response
 * @param response_len Size of response buffer
 * @return 0 on success, -1 on error
 */
static int handle_command(const char *json_cmd, char *response, size_t response_len)
{
    // Parse and execute command
    if (parse_json_command(json_cmd) < 0)
    {
        snprintf(response, response_len, "{\"status\":\"error\",\"message\":\"Invalid command\"}\n");
        return -1;
    }

    snprintf(response, response_len, "{\"status\":\"ok\"}\n");
    return 0;
}

/**
 * @brief Parse JSON command and update configuration
 *
 * Supported commands:
 * - {"type":"set_controller","controller":"balance","enabled":true}
 * - {"type":"set_pid","controller":"balance","kp":40.0,"ki":0.0,"kd":5.0}
 *   (balance and steering only — position has no gains, use set_pos_config)
 * - {"type":"set_pos_config","scale_d":30.0,"max_correction":5.0,...}
 * - {"type":"set_sbus_config","drive_channel":3,"drive_scale":0.10,...}
 * - {"type":"set_telemetry","encoders":true,"imu_full":false,...}
 * - {"type":"arm","value":true}
 * - {"type":"set_mode","value":1}
 *
 * @param json_cmd JSON command string
 * @return 0 on success, -1 on error
 */
static void *ipc_estop_reset_thread(void *arg)
{
    (void)arg;
    // Only clear the latch if the controller actually came back. Clearing it
    // unconditionally let ARM be accepted against a dead RoboClaw — the UI
    // showed the estop cleared while nothing downstream had recovered.
    if (motor_hal_roboclaw_reset() != 0)
    {
        LOG_ERROR("estop reset FAILED — RoboClaw did not reinitialise. "
                  "Staying latched; power-cycle the controller.");
        return NULL;
    }
    state.estop_latched = 0;
    state.trying = 0;  // let angle logic re-set cleanly
    LOG_INFO("estop reset complete — latch cleared");
    return NULL;
}

static int parse_json_command(const char *json_cmd)
{
    // Simple JSON parsing (in production, use json-c library)
    // For now, we'\''ll do basic string matching

    // Example: {"type":"set_controller","controller":"balance","enabled":true}
    if (strstr(json_cmd, "\"type\":\"set_controller\""))
    {
        if ((strstr(json_cmd, "\"controller\":\"pitch\"") || strstr(json_cmd, "\"controller\":\"balance\"")))
        {
            if (strstr(json_cmd, "\"enabled\":true"))
            {
                g_controllers.pitch = true;
                LOG_INFO("balance enabled");
            }
            else if (strstr(json_cmd, "\"enabled\":false"))
            {
                g_controllers.pitch = false;
                LOG_WARN("balance disabled - robot will fall!");
            }
        }
        else if (strstr(json_cmd, "\"controller\":\"position\""))
        {
            if (strstr(json_cmd, "\"enabled\":true"))
            {
                g_controllers.position = true;
                LOG_INFO("position enabled");
            }
            else
            {
                g_controllers.position = false;
                LOG_INFO("position disabled");
            }
        }
        else if ((strstr(json_cmd, "\"controller\":\"yaw\"") || strstr(json_cmd, "\"controller\":\"steering\"")))
        {
            if (strstr(json_cmd, "\"enabled\":true"))
            {
                g_controllers.yaw = true;
                LOG_INFO("steering enabled");
            }
            else
            {
                g_controllers.yaw = false;
                LOG_INFO("steering disabled");
            }
        }
        return 0;
    }

    // {"type":"set_telemetry","encoders":true,...}
    if (strstr(json_cmd, "\"type\":\"set_telemetry\""))
    {
        if (strstr(json_cmd, "\"encoders\":true"))
        {
            g_debug_config.telemetry.encoders = true;
        }
        else if (strstr(json_cmd, "\"encoders\":false"))
        {
            g_debug_config.telemetry.encoders = false;
        }

        if (strstr(json_cmd, "\"imu_full\":true"))
        {
            g_debug_config.telemetry.imu_full = true;
            LOG_WARN("imu_full enabled - HIGH BANDWIDTH");
        }
        else if (strstr(json_cmd, "\"imu_full\":false"))
        {
            g_debug_config.telemetry.imu_full = false;
        }

        if (strstr(json_cmd, "\"pid_states\":true"))
        {
            g_debug_config.telemetry.pid_states = true;
        }
        else if (strstr(json_cmd, "\"pid_states\":false"))
        {
            g_debug_config.telemetry.pid_states = false;
        }

        LOG_INFO("Telemetry configuration updated");
        return 0;
    }

    // {"type":"arm","value":true}  or  {"type":"arm","value":false}
    if (strstr(json_cmd, "\"type\":\"arm\""))
    {
        if (strstr(json_cmd, "\"value\":true"))
        {
            if (state.estop_latched)
            {
                LOG_WARN("ARM rejected — estop latched, press CLR ESTOP first");
            }
            else
            {
                state.armed = 1;
                // Set trying only if currently in bounds — fall detection manages it otherwise
                if (fabsf(state.theta - state.theta_offset) < 14.0f)
                    state.trying = 1;
                motor_hal_standby(0);
                rc_led_set(RC_LED_GREEN, 1);
                LOG_INFO("iPhone: ARMED (theta=%.2f trying=%d)", state.theta, state.trying);
            }
        }
        else
        {
            state.trying = 0;
            state.armed = 0;
            motor_hal_set_both(0.0f, 0.0f);
            motor_hal_standby(1);
            rc_led_set(RC_LED_GREEN, 0);
            LOG_INFO("iPhone: DISARMED");
        }
        return 0;
    }

    // {"type":"e_stop"} -- assert hardware e-stop
    if (strstr(json_cmd, "\"type\":\"e_stop\""))
    {
        state.armed        = 0;
        state.trying       = 0;
        state.estop_latched = 1;
        motor_hal_set_both(0.0f, 0.0f);
        motor_hal_standby(1);
        roboclaw_estop_assert();
        rc_led_set(RC_LED_GREEN, 0);
        LOG_INFO("iPhone: E-STOP asserted");
        return 0;
    }

    // {"type":"reset_estop"} -- WriteNVM reset in background thread
    if (strstr(json_cmd, "\"type\":\"reset_estop\""))
    {
        pthread_t tid;
        pthread_create(&tid, NULL, ipc_estop_reset_thread, NULL);
        pthread_detach(tid);
        LOG_INFO("iPhone: estop reset started");
        return 0;
    }

    // {"type":"debug_position","value":true}  -- toggle verbose position hold logging
    if (strstr(json_cmd, "\"type\":\"debug_position\""))
    {
        g_debug_config.debug_position = strstr(json_cmd, "\"value\":true") != NULL;
        LOG_INFO("position hold verbose debug: %s", g_debug_config.debug_position ? "ON" : "OFF");
        return 0;
    }

    // {"type":"zero_imu"} — zero pitch offset + encoders, save all config
    if (strstr(json_cmd, "\"type\":\"zero_imu\""))
    {
        // pitch_offset and state.theta are both DEGREES, despite the old
        // comment here claiming radians — imu_config.c computes it with
        // RAD_TO_DEG and subtracts it from pitch_deg. The code was right, the
        // comment was not, and it is now in robot.conf where the units are
        // documented next to the value.
        g_imu_offsets.pitch_offset -= state.theta;
        state.theta_offset = 0.0f;
        // Also zero encoders so position hold starts from a clean position
        motor_hal_encoder_reset_all();
        state.enc_left = 0;
        state.enc_right = 0;
        state.phi_left = 0.0f;
        state.phi_right = 0.0f;
        state.enc_pos = 0;
        state.enc_pos_target = 0;
        state.enc_velocity = 0;
        state.enc_vel_reset = 1;
        robot_config_save_current(NULL);
        LOG_INFO("iPhone: IMU zeroed + encoders reset, saved");
        return 0;
    }

    // {"type":"zero_encoders"} — reset encoder position only
    if (strstr(json_cmd, "\"type\":\"zero_encoders\""))
    {
        motor_hal_encoder_reset_all();
        state.enc_left = 0;
        state.enc_right = 0;
        state.phi_left = 0.0f;
        state.phi_right = 0.0f;
        state.enc_pos = 0;
        state.enc_pos_target = 0;
        state.enc_velocity = 0;
        state.enc_vel_reset = 1;
        LOG_INFO("iPhone: encoders zeroed");
        return 0;
    }

    // {"type":"set_theta_offset","value":1.5}  — runtime balance trim, saved to pidconfig.txt
    if (strstr(json_cmd, "\"type\":\"set_theta_offset\""))
    {
        const char *p = strstr(json_cmd, "\"value\":");
        if (!p)
            return -1;
        float val = 0.0f;
        if (sscanf(p, "\"value\":%f", &val) != 1)
            return -1;
        if (val > 30.0f)
            val = 30.0f;
        if (val < -30.0f)
            val = -30.0f;
        state.theta_offset = val;
        robot_config_save_current(NULL);
        LOG_INFO("iPhone: theta_offset = %.2f deg, saved", val);
        return 0;
    }

    // {"type":"set_mode","value":1}  0=idle 1=balance 2=ext_input 3=manual
    if (strstr(json_cmd, "\"type\":\"set_mode\""))
    {
        const char *p = strstr(json_cmd, "\"value\":");
        if (!p)
            return -1;
        int val = 0;
        if (sscanf(p, "\"value\":%d", &val) != 1)
            return -1;
        if (val < 0 || val > 3)
            return -1;
        state.mode = (robot_mode_t)val;
        LOG_INFO("iPhone: mode → %d", val);
        return 0;
    }

    // {"type":"set_pid","controller":"balance","kp":40.0,"ki":0.5,"kd":5.0}
    // {"type":"set_pid","controller":"steering","kp":1.0,"ki":0.0,"kd":0.1}
    // balance and steering only. position is not a PID — it is tuned via set_pos_config.
    if (strstr(json_cmd, "\"type\":\"set_pid\""))
    {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        const char *p;

        p = strstr(json_cmd, "\"kp\":");
        if (p)
            sscanf(p, "\"kp\":%f", &kp);
        p = strstr(json_cmd, "\"ki\":");
        if (p)
            sscanf(p, "\"ki\":%f", &ki);
        p = strstr(json_cmd, "\"kd\":");
        if (p)
            sscanf(p, "\"kd\":%f", &kd);

        if ((strstr(json_cmd, "\"controller\":\"pitch\"") || strstr(json_cmd, "\"controller\":\"balance\"")))
        {
            pid_set_gains(&pitch_pid, kp, ki, kd);
            pid_reset(&pitch_pid);
            LOG_INFO("iPhone: balance kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
            return 0;
        }
        if (strstr(json_cmd, "\"controller\":\"position\""))
        {
            // position hold has no PID gains — it is a zone-based position hold. Accepting
            // this silently (as the previous revision did) made it look like the
            // gains were being applied. Reject it and point at the real knobs.
            LOG_WARN("set_pid: position is not a PID and has no kp/ki/kd. "
                     "Use set_pos_config (zone_a/b/c, scale_a/b/c/d, "
                     "max_correction, max_angle_rate) instead.");
            return -1;
        }
        if ((strstr(json_cmd, "\"controller\":\"yaw\"") || strstr(json_cmd, "\"controller\":\"steering\"")))
        {
            pid_set_gains(&yaw_pid, kp, ki, kd);
            pid_reset(&yaw_pid);
            LOG_INFO("iPhone: steering kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
            return 0;
        }
        LOG_WARN("set_pid: unknown controller in: %s", json_cmd);
        return -1;
    }

    // {"type":"save_pid"}  -- snapshot every live setting into robot.conf
    if (strstr(json_cmd, "\"type\":\"save_pid\""))
    {
        if (robot_config_save_current(NULL) == 0)
            LOG_INFO("iPhone: config saved to robot.conf");
        else
            LOG_WARN("iPhone: failed to save config");
        return 0;
    }

    // {"type":"set_pos_config",...} -- update position controller params at runtime
    // {"type":"nudge","axis":"pitch|yaw|fwd","delta":0.05}
    //
    // Bounded relative adjustments for the dashboard's arrow controls. Relative
    // and bounded on purpose: a typed absolute value or a dragged slider can put
    // a balancing robot somewhere violent in one action, and there is no undo.
    if (strstr(json_cmd, "\"type\":\"nudge\""))
    {
        float d = 0.0f;
        const char *dp = strstr(json_cmd, "\"delta\":");
        if (dp)
            sscanf(dp + strlen("\"delta\":"), "%f", &d);

        /* Hard per-press ceiling, per axis. Whatever the UI believes its step
         * size to be, one press can never move the robot further than this.
         * pitch is degrees, yaw is degrees of phi_diff, fwd is encoder ticks —
         * so they cannot share one limit. */
        float lim = 2.0f;                                   /* pitch: 2 deg    */
        if (strstr(json_cmd, "\"axis\":\"pose\"")) lim = 0.5f;     /* pose: fine only */
        if (strstr(json_cmd, "\"axis\":\"yaw\"")) lim = 45.0f;  /* ~35 deg yaw     */
        else if (strstr(json_cmd, "\"axis\":\"fwd\"")) lim = 200.0f; /* ~335 mm    */
        if (d > lim) d = lim;
        if (d < -lim) d = -lim;

        if (strstr(json_cmd, "\"axis\":\"pitch\""))
        {
            float v = state.theta_offset + d;
            if (v > 30.0f) v = 30.0f;
            if (v < -30.0f) v = -30.0f;
            state.theta_offset = v;
            LOG_INFO("nudge pitch: theta_offset = %.3f deg", v);
        }
        else if (strstr(json_cmd, "\"axis\":\"pose\""))
        {
            /* Commanded lean, degrees. Small ceiling — this makes the robot
             * drive away, and it is meant for observation at fractions of a
             * degree, not for driving. */
            float v = state.pose_lean + d;
            if (v > 3.0f) v = 3.0f;
            if (v < -3.0f) v = -3.0f;
            state.pose_lean = v;
            LOG_INFO("nudge pose: commanded lean = %.3f deg", v);
            return 0; /* transient, not persisted */
        }
        else if (strstr(json_cmd, "\"axis\":\"pose_zero\""))
        {
            state.pose_lean = 0.0f;
            LOG_INFO("pose cleared");
            return 0;
        }
        else if (strstr(json_cmd, "\"axis\":\"yaw\""))
        {
            /* Degrees of phi_diff, same units as the steering setpoint. */
            state.yaw += d;
            LOG_INFO("nudge yaw: steering target = %.2f", state.yaw);
            return 0; /* transient, not persisted */
        }
        else if (strstr(json_cmd, "\"axis\":\"fwd\""))
        {
            /* Encoder ticks. Moves the position hold's target, so the bot drives
             * there under the hold loop rather than by a raw lean command. */
            state.enc_pos_target += (int32_t)d;
            LOG_INFO("nudge fwd: enc_pos_target = %d", state.enc_pos_target);
            return 0; /* transient, not persisted */
        }
        else
        {
            LOG_WARN("nudge: unknown axis in %s", json_cmd);
            return -1;
        }

        robot_config_save_current(NULL);
        return 0;
    }

    // {"type":"set_sbus_config","drive_channel":3,"drive_scale":0.10,...}
    //
    // Transmitter mapping, live. Every field optional. Exists because working
    // out which physical stick is on which channel, which way round it is, and
    // how hard it should push is a two-second experiment with a slider and an
    // hour of guesswork without one.
    if (strstr(json_cmd, "\"type\":\"set_sbus_config\""))
    {
        sbus_config_t cfg = g_sbus_config;

#define SCFG_F(key, field)                                         \
    do                                                             \
    {                                                              \
        const char *_p = strstr(json_cmd, "\"" key "\":");         \
        if (_p)                                                    \
            sscanf(_p + strlen("\"" key "\":"), "%f", &cfg.field); \
    } while (0)
#define SCFG_I(key, field)                                         \
    do                                                             \
    {                                                              \
        const char *_p = strstr(json_cmd, "\"" key "\":");         \
        if (_p)                                                    \
        {                                                          \
            float _v = 0;                                          \
            sscanf(_p + strlen("\"" key "\":"), "%f", &_v);        \
            cfg.field = (int)_v;                                   \
        }                                                          \
    } while (0)

        SCFG_I("drive_channel", drive_channel);
        SCFG_I("turn_channel", turn_channel);
        SCFG_F("drive_scale", drive_scale);
        SCFG_F("turn_scale", turn_scale);
        SCFG_F("turn_rate", turn_rate);
        SCFG_I("drive_invert", drive_invert);
        SCFG_I("turn_invert", turn_invert);
        SCFG_F("deadband", deadband);
        SCFG_I("require_center", require_center);

#undef SCFG_F
#undef SCFG_I

        /* Clamp rather than reject: a bad channel number from a slider should
         * not leave the robot with no drive input at all. */
        if (cfg.drive_channel < 1 || cfg.drive_channel > 16) cfg.drive_channel = 2;
        if (cfg.turn_channel < 1 || cfg.turn_channel > 16) cfg.turn_channel = 1;
        if (cfg.drive_scale < 0.0f) cfg.drive_scale = 0.0f;
        if (cfg.drive_scale > 1.0f) cfg.drive_scale = 1.0f;
        if (cfg.turn_scale < 0.0f) cfg.turn_scale = 0.0f;
        if (cfg.turn_scale > 1.0f) cfg.turn_scale = 1.0f;
        if (cfg.turn_rate < 0.0f) cfg.turn_rate = 0.0f;
        if (cfg.turn_rate > 1000.0f) cfg.turn_rate = 1000.0f;
        if (cfg.deadband < 0.0f) cfg.deadband = 0.0f;
        if (cfg.deadband > 0.5f) cfg.deadband = 0.5f;

        g_sbus_config = cfg;
        LOG_INFO("sbus_config: drive=CH%d x%.3f inv=%d  turn=CH%d x%.3f inv=%d  "
                 "db=%.3f req_center=%d",
                 cfg.drive_channel, cfg.drive_scale, cfg.drive_invert,
                 cfg.turn_channel, cfg.turn_scale, cfg.turn_invert,
                 cfg.deadband, cfg.require_center);

        if (robot_config_save_current(NULL) != 0)
            LOG_WARN("sbus_config applied but could not be saved to disk");
        ipc_config_touch();
        return 0;
    }

    if (strstr(json_cmd, "\"type\":\"set_pos_config\""))
    {
        pos_config_t cfg = g_pos_config;

#define PCFG_FLOAT(key, field)                                     \
    do                                                             \
    {                                                              \
        const char *_p = strstr(json_cmd, "\"" key "\":");         \
        if (_p)                                                    \
            sscanf(_p + strlen("\"" key "\":"), "%f", &cfg.field); \
    } while (0)
#define PCFG_INT(key, field)                                         \
    do                                                               \
    {                                                                \
        const char *_p = strstr(json_cmd, "\"" key "\":");           \
        if (_p)                                                      \
        {                                                            \
            int _v;                                                  \
            if (sscanf(_p + strlen("\"" key "\":"), "%d", &_v) == 1) \
                cfg.field = _v;                                      \
        }                                                            \
    } while (0)

        PCFG_INT("zone_a", zone_a);
        PCFG_INT("zone_b", zone_b);
        PCFG_INT("zone_c", zone_c);
        PCFG_FLOAT("scale_a", scale_a);
        PCFG_FLOAT("scale_b", scale_b);
        PCFG_FLOAT("scale_c", scale_c);
        PCFG_FLOAT("scale_d", scale_d);
        PCFG_FLOAT("vel_scale_stop", vel_scale_stop);
        PCFG_FLOAT("vel_scale_move", vel_scale_move);
        PCFG_FLOAT("vel_scale_turning", vel_scale_turning);
        PCFG_INT("stopped_vel", stopped_vel);
        PCFG_FLOAT("max_correction", max_correction);
        PCFG_FLOAT("max_angle_rate", max_angle_rate);
        PCFG_INT("back_to_spot", back_to_spot);
        PCFG_INT("drive_mode", drive_mode);
        PCFG_FLOAT("drive_rate", drive_rate);
        PCFG_INT("runaway_limit", runaway_limit);

#undef PCFG_FLOAT
#undef PCFG_INT

        pos_config_apply(&cfg);

        // Persist immediately. Previously this handler applied but never saved,
        // so a tuning session only reached pidconfig.txt if the operator happened
        // to fire zero_imu / set_theta_offset / set_pid afterwards — each of which
        // saves the whole config as a side effect. Tune pos_config alone and
        // restart, and the entire session was silently lost.
        // Whole-file atomic write. The old pos_config_save() opened the file
        // in append mode, so calling it alone — as this handler does — tacked a
        // duplicate [position] block on every single slider move.
        if (robot_config_save_current(NULL) != 0)
            LOG_WARN("pos_config updated but could not be saved to disk");
        LOG_INFO("iPhone: pos_config updated + saved");
        ipc_config_touch();
        return 0;
    }

    // {"type":"set_motor_config",...} -- update RoboClaw drive mode + velocity params at runtime
    // Fields (all optional — omit any you don't want to change):
    //   mode       0=duty  1=velocity  2=velocity+accel
    //   qpps_max   top motor speed in encoder pulses/sec at full throttle
    //   accel_qpps acceleration ramp rate (pulses/s^2), mode 2 only
    //   pol_l      left  motor polarity: +1.0 or -1.0
    //   pol_r      right motor polarity: +1.0 or -1.0
    if (strstr(json_cmd, "\"type\":\"set_motor_config\""))
    {
        motor_config_t cfg = g_motor_config;

#define MCFG_INT(key, field)                                         \
    do                                                               \
    {                                                                \
        const char *_p = strstr(json_cmd, "\"" key "\":");           \
        if (_p)                                                      \
        {                                                            \
            int _v;                                                  \
            if (sscanf(_p + strlen("\"" key "\":"), "%d", &_v) == 1) \
                cfg.field = _v;                                      \
        }                                                            \
    } while (0)
#define MCFG_FLOAT(key, field)                                     \
    do                                                             \
    {                                                              \
        const char *_p = strstr(json_cmd, "\"" key "\":");         \
        if (_p)                                                    \
            sscanf(_p + strlen("\"" key "\":"), "%f", &cfg.field); \
    } while (0)

        MCFG_INT("mode", mode);
        MCFG_INT("qpps_max", qpps_max);
        MCFG_INT("accel_qpps", accel_qpps);
        MCFG_FLOAT("pol_l", pol_l);
        MCFG_FLOAT("pol_r", pol_r);
        MCFG_FLOAT("enc_pol_l", enc_pol_l);
        MCFG_FLOAT("enc_pol_r", enc_pol_r);

        /* Baud rate change: close and reopen serial at new rate */
        {
            int old_baud = cfg.baud;
            MCFG_INT("baud", baud);
            if (cfg.baud != old_baud && cfg.baud > 0)
            {
                LOG_INFO("set_motor_config: baud change %d → %d", old_baud, cfg.baud);
                if (motor_hal_set_baud(cfg.baud) != 0)
                {
                    LOG_WARN("set_motor_config: baud change failed, reverting");
                    cfg.baud = old_baud;
                }
            }
        }

#undef MCFG_INT
#undef MCFG_FLOAT

        if (cfg.mode < 0 || cfg.mode > 2)
        {
            LOG_WARN("set_motor_config: invalid mode %d (0-2 only)", cfg.mode);
            return -1;
        }
        if (cfg.qpps_max < 1)
            cfg.qpps_max = 1;
        if (cfg.accel_qpps < 1)
            cfg.accel_qpps = 1;

        motor_config_apply(&cfg);
        LOG_INFO("iPhone: motor_config updated — mode=%d qpps_max=%d accel=%d baud=%d pol=%.1f/%.1f",
                 cfg.mode, cfg.qpps_max, cfg.accel_qpps, cfg.baud, cfg.pol_l, cfg.pol_r);
        ipc_config_touch();
        return 0;
    }

    // {"type":"reboot_app"}
    // Schedules a systemd service restart 1 second from now.
    // The IPC reply is sent before the process dies.
    if (strstr(json_cmd, "\"type\":\"reboot_app\""))
    {
        LOG_INFO("reboot_app: scheduling service restart in 1 s");
        system("(sleep 1 && systemctl restart balance_bot) &");
        return 0;
    }

    // {"type":"set_claw_pid","kp":1.0,"ki":0.5,"kd":0.25}
    // Push velocity PID gains to the RoboClaw (M1 + M2).
    // Only has effect in velocity modes (mode 1 or 2).
    // Values are NOT written to NVM automatically — use save_pid to persist.
    if (strstr(json_cmd, "\"type\":\"set_claw_pid\""))
    {
        float kp = g_motor_config.claw_kp;
        float ki = g_motor_config.claw_ki;
        float kd = g_motor_config.claw_kd;

#define CPID_FLOAT(key, var)                                           \
    do {                                                               \
        const char *_p = strstr(json_cmd, "\"" key "\":");             \
        if (_p) sscanf(_p + strlen("\"" key "\":"), "%f", &(var));     \
    } while (0)

        CPID_FLOAT("kp", kp);
        CPID_FLOAT("ki", ki);
        CPID_FLOAT("kd", kd);

#undef CPID_FLOAT

        if (kp < 0.0f || ki < 0.0f || kd < 0.0f)
        {
            LOG_WARN("set_claw_pid: gains must be >= 0");
            return -1;
        }
        if (motor_hal_set_claw_pid(kp, ki, kd) != 0)
        {
            LOG_WARN("set_claw_pid: hardware write failed");
            return -1;
        }
        LOG_INFO("iPhone: claw PID set — kp=%.4f ki=%.4f kd=%.4f", kp, ki, kd);
        /* motor_hal_set_claw_pid() pushes the gains to the RoboClaw but does not
         * touch g_motor_config, so the config packet would keep advertising the
         * old values to any client that connected afterwards. Mirror them here
         * before flagging the resend. */
        g_motor_config.claw_kp = kp;
        g_motor_config.claw_ki = ki;
        g_motor_config.claw_kd = kd;
        ipc_config_touch();
        return 0;
    }

    LOG_WARN("Unknown command type");
    return -1;
}

/**
 * @brief Build JSON telemetry packet
 *
 * Constructs a JSON string containing all enabled telemetry data.
 * Only includes fields that are enabled in g_debug_config.
 *
 * @param buffer Output buffer for JSON string
 * @param size Size of output buffer
 */
/**
 * @brief Bounds-safe append used by build_telemetry_json().
 *
 * Replaces the idiom
 *     pos = json_append(buffer, pos, size, ...);
 * which is unsafe: snprintf returns the length it WOULD have written, so on
 * truncation pos runs past size. pos is size_t, so the next call computes
 * (size - pos) as a huge unsigned value and snprintf writes past the end of
 * the buffer. This clamps pos at size instead, so an oversized payload
 * truncates cleanly and every later append becomes a no-op.
 */
__attribute__((format(printf, 4, 5)))
static size_t json_append(char *buf, size_t pos, size_t size, const char *fmt, ...)
{
    if (pos >= size)
        return size;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + pos, size - pos, fmt, ap);
    va_end(ap);
    if (n < 0)
        return pos;
    pos += (size_t)n;
    return (pos > size) ? size : pos;
}

static void build_telemetry_json(char *buffer, size_t size)
{
    size_t pos = 0;

    pos = json_append(buffer, pos, size, "{");
    pos = json_append(buffer, pos, size, "\"type\":\"telemetry\",");
    pos = json_append(buffer, pos, size, "\"timestamp\":%llu,",
                    (unsigned long long)(rc_nanos_since_boot() / 1000));

    // System status (always included)
    if (g_debug_config.telemetry.system_status)
    {
        pos = json_append(buffer, pos, size,
                        "\"system\":{\"battery\":%.2f,\"armed\":%s,\"mode\":%d,\"loop_hz\":%.1f,\"theta_offset\":%.4f,\"pose_lean\":%.4f,"
                        "\"batt_voltage\":%.3f,\"batt_status\":%d,\"claw_voltage\":%.2f,\"claw_temp\":%.1f,"
                        /* Telemetry packets the kernel refused because the
                         * client was not draining. Non-zero means holes in the
                         * graphs, and it means the bridge or the link is behind
                         * -- not the renderer. */
                        "\"tx_drops\":%lu},",
                        g_telemetry_data.system.battery_voltage,
                        g_telemetry_data.system.armed ? "true" : "false",
                        g_telemetry_data.system.mode,
                        g_telemetry_data.system.loop_hz,
                        state.theta_offset,
                        state.pose_lean,
                        g_telemetry_data.system.batt_voltage,
                        (int)g_telemetry_data.system.batt_status,
                        g_telemetry_data.system.claw_voltage,
                        g_telemetry_data.system.claw_temp,
                        ipc_get_tx_drops());
    }

    /* motor_config / pos_config / sbus_config used to be rebuilt and appended
     * here on every packet. They only change when a set_* command changes them,
     * so ~700 of the ~2000 bytes in every packet were byte-identical to the
     * previous packet. They now live in the "config" message, sent once when a
     * client connects and again whenever a command actually changes one of
     * them. See build_config_json() / ipc_broadcast_config(). */

    // Encoders
    if (g_debug_config.telemetry.encoders)
    {
        pos = json_append(buffer, pos, size,
                        "\"encoders\":{\"left_ticks\":%d,\"right_ticks\":%d,"
                        "\"left_rad\":%.3f,\"right_rad\":%.3f,"
                        "\"left_vel\":%.3f,\"right_vel\":%.3f},",
                        g_telemetry_data.encoders.left_ticks,
                        g_telemetry_data.encoders.right_ticks,
                        g_telemetry_data.encoders.left_rad,
                        g_telemetry_data.encoders.right_rad,
                        g_telemetry_data.encoders.left_vel,
                        g_telemetry_data.encoders.right_vel);
    }

    // IMU attitude (for 3D visualization)
    if (g_debug_config.telemetry.imu_attitude)
    {
        pos = json_append(buffer, pos, size,
                        "\"imu\":{\"theta\":%.4f,\"phi\":%.4f,\"psi\":%.4f,"
                        "\"theta_dot\":%.4f,\"phi_dot\":%.4f,\"psi_dot\":%.4f,"
                        "\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f",
                        g_telemetry_data.imu.theta,
                        g_telemetry_data.imu.phi,
                        g_telemetry_data.imu.psi,
                        g_telemetry_data.imu.theta_dot,
                        g_telemetry_data.imu.phi_dot,
                        g_telemetry_data.imu.psi_dot,
                        g_telemetry_data.imu.qw,
                        g_telemetry_data.imu.qx,
                        g_telemetry_data.imu.qy,
                        g_telemetry_data.imu.qz);

        // Add full IMU data if enabled
        if (g_debug_config.telemetry.imu_full)
        {
            pos = json_append(buffer, pos, size,
                            ",\"accel_x\":%.3f,\"accel_y\":%.3f,\"accel_z\":%.3f,"
                            "\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f",
                            g_telemetry_data.imu.accel_x,
                            g_telemetry_data.imu.accel_y,
                            g_telemetry_data.imu.accel_z,
                            g_telemetry_data.imu.gyro_x,
                            g_telemetry_data.imu.gyro_y,
                            g_telemetry_data.imu.gyro_z);
        }

        pos = json_append(buffer, pos, size, "},");
    }

    // PID states
    if (g_debug_config.telemetry.pid_states)
    {
        pos = json_append(buffer, pos, size,
                        "\"balance\":{\"enabled\":%s,\"setpoint\":%.4f,"
                        "\"measurement\":%.4f,\"error\":%.4f,\"output\":%.4f,"
                        "\"p_term\":%.4f,\"i_term\":%.4f,\"d_term\":%.4f,"
                        "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},",
                        g_telemetry_data.pitch.enabled ? "true" : "false",
                        g_telemetry_data.pitch.setpoint,
                        g_telemetry_data.pitch.measurement,
                        g_telemetry_data.pitch.error,
                        g_telemetry_data.pitch.output,
                        g_telemetry_data.pitch.p_term,
                        g_telemetry_data.pitch.i_term,
                        g_telemetry_data.pitch.d_term,
                        g_telemetry_data.pitch.kp,
                        g_telemetry_data.pitch.ki,
                        g_telemetry_data.pitch.kd);

        // position hold is a zone-based position hold, not a PID. Its keys deliberately
        // do not match balance/steering — consumers must not treat them interchangeably.
        // "kind" is emitted so a client can branch on it instead of assuming.
        pos = json_append(buffer, pos, size,
                        "\"position\":{\"enabled\":%s,\"kind\":\"zone_position_hold\","
                        "\"enc_pos_target\":%d,\"enc_pos\":%d,\"enc_error\":%d,"
                        "\"enc_velocity\":%.3f,\"pos_correction\":%.4f,\"vel_damp\":%.4f,"
                        "\"theta_ref_adj\":%.4f,\"active_scale\":%.4f,"
                        "\"max_correction\":%.4f},",
                        g_telemetry_data.position.enabled ? "true" : "false",
                        (int)g_telemetry_data.position.enc_pos_target,
                        (int)g_telemetry_data.position.enc_pos,
                        (int)g_telemetry_data.position.enc_error,
                        g_telemetry_data.position.enc_velocity,
                        g_telemetry_data.position.pos_correction,
                        g_telemetry_data.position.vel_damp,
                        g_telemetry_data.position.theta_ref_adj,
                        g_telemetry_data.position.active_scale,
                        g_telemetry_data.position.max_correction);

        pos = json_append(buffer, pos, size,
                        "\"steering\":{\"enabled\":%s,\"setpoint\":%.4f,"
                        "\"measurement\":%.4f,\"error\":%.4f,\"output\":%.4f,"
                        "\"p_term\":%.4f,\"i_term\":%.4f,\"d_term\":%.4f,"
                        "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},",
                        g_telemetry_data.yaw.enabled ? "true" : "false",
                        g_telemetry_data.yaw.setpoint,
                        g_telemetry_data.yaw.measurement,
                        g_telemetry_data.yaw.error,
                        g_telemetry_data.yaw.output,
                        g_telemetry_data.yaw.p_term,
                        g_telemetry_data.yaw.i_term,
                        g_telemetry_data.yaw.d_term,
                        g_telemetry_data.yaw.kp,
                        g_telemetry_data.yaw.ki,
                        g_telemetry_data.yaw.kd);
    }

    // Cat position
    if (g_debug_config.telemetry.ext_input && g_telemetry_data.ext_input.valid)
    {
        pos = json_append(buffer, pos, size,
                        "\"ext_input\":{\"valid\":true,\"x\":%.3f,\"y\":%.3f,\"confidence\":%.2f},",
                        g_telemetry_data.ext_input.x,
                        g_telemetry_data.ext_input.y,
                        g_telemetry_data.ext_input.confidence);
    }

    // motors -- per-wheel duty as actually commanded. Written by the control
    // loop into g_telemetry_data.motors. Without this block left/right duty
    // reached the OLED and nothing else, leaving any drive-side asymmetry
    // completely unobservable from a log.
    pos = json_append(buffer, pos, size,
                    "\"motors\":{\"left_duty\":%.4f,\"right_duty\":%.4f},",
                    g_telemetry_data.motors.left_duty,
                    g_telemetry_data.motors.right_duty);

    /* sbus moved out of this packet entirely -- see build_rc_json(). It was the
     * one block here whose source changes faster than this packet is sent: the
     * receiver delivers a frame every ~7 ms and this packet goes out every
     * 100 ms, so the RC panel was being fed 1 sample in 14 and could not show a
     * stick movement faithfully no matter how fast the browser drew it. */

    /* Remove trailing comma. pos == 0 is impossible (the "{" above always
     * lands), but read the guard as belt-and-braces if a future edit makes the
     * first append conditional. */
    if (pos > 0 && buffer[pos - 1] == ',')
        pos--;

    pos = json_append(buffer, pos, size, "}\n");
}

/**
 * @brief Build the RC (SBUS) packet.
 *
 * Split out of build_telemetry_json() so it can be broadcast at its own, much
 * higher rate. input_sbus.c already decodes all 16 channels and both flags;
 * this mirrors what draw_sbus() shows in the ncurses UI.
 *
 * Raw channel values are 172..1811, centre 992 (see SBUS_MIN/MID/MAX_RAW).
 * Decoded fields are what the firmware actually derived from them, so the
 * dashboard can show the mapping working rather than just the numbers.
 *
 * Roughly 270 bytes, versus ~2000 for a full telemetry packet -- which is what
 * makes sending it at 50 Hz cheaper than the old 10 Hz combined packet was.
 */
static void build_rc_json(char *buffer, size_t size)
{
    size_t pos = 0;

    pos = json_append(buffer, pos, size,
                    "{\"type\":\"rc\",\"timestamp\":%llu,"
                    "\"connected\":%s,\"failsafe\":%s,\"drive_armed\":%s,\"ch\":[",
                    (unsigned long long)(rc_nanos_since_boot() / 1000),
                    sbus_is_connected() ? "true" : "false",
                    sbus_get_failsafe() ? "true" : "false",
                    sbus_drive_armed() ? "true" : "false");
    for (int i = 0; i < 16; i++)
        pos = json_append(buffer, pos, size,
                        "%s%u", i ? "," : "", (unsigned)sbus_get_channel_raw(i));
    pos = json_append(buffer, pos, size,
                    "],\"drive\":%.4f,\"turn\":%.4f,"
                    "\"arm\":%d,\"kill\":%d,\"speed\":%d,"
                    "\"sw_c\":%d,\"sw_e\":%d,\"sw_f\":%s,"
                    "\"aux1\":%.4f,\"aux2\":%.4f}\n",
                    sbus_get_drive(), sbus_get_turn(),
                    sbus_get_arm(), sbus_get_kill(), sbus_get_speed_mode(),
                    sbus_get_sw_c(), sbus_get_sw_e(),
                    sbus_get_sw_f() ? "true" : "false",
                    sbus_get_aux1(), sbus_get_aux2());
}

/**
 * @brief Build the config packet -- everything that only changes on command.
 *
 * Sent once when a client connects and again whenever a set_* command actually
 * changes one of these values (see ipc_config_touch()). A client that has never
 * seen one of these has no values to sync its controls to, so this must be sent
 * on connect and not merely on change.
 */
static void build_config_json(char *buffer, size_t size)
{
    size_t pos = 0;

    pos = json_append(buffer, pos, size,
                    "{\"type\":\"config\",\"timestamp\":%llu,",
                    (unsigned long long)(rc_nanos_since_boot() / 1000));

    pos = json_append(buffer, pos, size,
                    "\"motor_config\":{\"mode\":%d,\"qpps_max\":%d,\"accel_qpps\":%d,"
                    "\"pol_l\":%.1f,\"pol_r\":%.1f,\"enc_pol_l\":%.1f,\"enc_pol_r\":%.1f,"
                    "\"claw_kp\":%.6f,\"claw_ki\":%.6f,\"claw_kd\":%.6f,\"baud\":%d},",
                    g_motor_config.mode,
                    g_motor_config.qpps_max,
                    g_motor_config.accel_qpps,
                    g_motor_config.pol_l,
                    g_motor_config.pol_r,
                    g_motor_config.enc_pol_l,
                    g_motor_config.enc_pol_r,
                    g_motor_config.claw_kp,
                    g_motor_config.claw_ki,
                    g_motor_config.claw_kd,
                    g_motor_config.baud);

    pos = json_append(buffer, pos, size,
                    "\"pos_config\":{"
                    "\"zone_a\":%d,\"zone_b\":%d,\"zone_c\":%d,"
                    "\"scale_a\":%.1f,\"scale_b\":%.1f,\"scale_c\":%.1f,\"scale_d\":%.1f,"
                    "\"vel_scale_stop\":%.1f,\"vel_scale_move\":%.1f,\"vel_scale_turning\":%.1f,"
                    "\"stopped_vel\":%d,\"max_correction\":%.2f,"
                    "\"max_angle_rate\":%.2f,\"back_to_spot\":%d,"
                    "\"drive_mode\":%d,\"drive_rate\":%.1f,\"runaway_limit\":%d},",
                    g_pos_config.zone_a, g_pos_config.zone_b, g_pos_config.zone_c,
                    g_pos_config.scale_a, g_pos_config.scale_b,
                    g_pos_config.scale_c, g_pos_config.scale_d,
                    g_pos_config.vel_scale_stop, g_pos_config.vel_scale_move,
                    g_pos_config.vel_scale_turning,
                    g_pos_config.stopped_vel, g_pos_config.max_correction,
                    g_pos_config.max_angle_rate, g_pos_config.back_to_spot,
                    g_pos_config.drive_mode, g_pos_config.drive_rate,
                    g_pos_config.runaway_limit);

    pos = json_append(buffer, pos, size,
                      "\"sbus_config\":{\"drive_channel\":%d,\"turn_channel\":%d,"
                      "\"drive_scale\":%.4f,\"turn_scale\":%.4f,\"turn_rate\":%.2f,"
                      "\"drive_invert\":%d,\"turn_invert\":%d,"
                      "\"deadband\":%.4f,\"require_center\":%d}",
                      g_sbus_config.drive_channel, g_sbus_config.turn_channel,
                      g_sbus_config.drive_scale, g_sbus_config.turn_scale,
                      g_sbus_config.turn_rate,
                      g_sbus_config.drive_invert, g_sbus_config.turn_invert,
                      g_sbus_config.deadband, g_sbus_config.require_center);

    pos = json_append(buffer, pos, size, "}\n");
}

/**
 * @brief Send one already-built, newline-terminated packet to every client.
 *
 * A unix socket is a BYTE STREAM with no message boundaries: the reader gets
 * whatever chunk sizes the kernel chooses, which may split a packet in half or
 * glue two together. Without a delimiter the far end cannot tell where one JSON
 * object ends and the next begins, so it guesses -- and every guess it gets
 * wrong is a silently dropped packet. Hence exactly one '\n' per packet, and
 * one write() per packet so the newline cannot be separated from the object it
 * terminates.
 *
 * The write is non-blocking (see ipc_server_init / the accept path, which set
 * O_NONBLOCK). This matters more than it looks: this function runs on the
 * CONTROL LOOP thread. With a blocking fd, a client that stops draining -- a
 * phone that walked out of wifi range, a paused node bridge -- fills the socket
 * buffer and then write() blocks the balance loop until it drains. Dropping a
 * telemetry frame is free; stalling the loop that keeps the robot upright is
 * not. EAGAIN therefore means "skip this frame for this client", not an error.
 */
/* Send buffer headroom reserved for telemetry, in bytes.
 *
 * Telemetry and RC share one socket buffer (~208 KB, which in practice accepts
 * ~90 packets of this size). Before the RC stream existed, 10 packets/s meant a
 * stalled reader was tolerated for ~9 s. At 10 + 50 packets/s that headroom
 * collapses to ~1.6 s -- and when it runs out, the kernel refuses whatever is
 * written next, which is as likely to be a telemetry packet as an RC one.
 *
 * That is what makes graph traces go angular: a dropped telemetry packet is a
 * missing sample, and the plot joins the two survivors with a straight line.
 * RC is resent 50 times a second and nobody can see one missing frame; a
 * telemetry sample is gone from the trace and from any CSV recorded from it.
 *
 * So RC yields. If the queue is deeper than this, RC packets are skipped and
 * the remaining buffer is left for telemetry. */
#define TX_RESERVE_BYTES (64 * 1024)

/* true if this client's send queue is shallow enough to accept a low priority
 * packet. Checked before writing rather than discovering it via EAGAIN, so a
 * low priority packet cannot consume the last of the buffer.
 *
 * SIOCOUTQ is Linux-only, and this tree is edited on a Mac and built on the
 * BeagleBone. Guarded so a local `make` on macOS still compiles rather than
 * failing on a missing <linux/sockios.h>; there it degrades to the unprioritised
 * behaviour, which is correct-but-worse, not broken. */
static bool client_has_headroom(int fd)
{
#ifdef SIOCOUTQ
    int queued = 0;
    if (ioctl(fd, SIOCOUTQ, &queued) != 0)
        return true;            /* cannot tell -- behave as before */
    return queued < TX_RESERVE_BYTES;
#else
    (void)fd;
    return true;
#endif
}

static void ipc_broadcast_line(const char *buffer, size_t len, const char *what,
                               bool low_priority)
{
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (!clients[i].active)
            continue;

        if (low_priority && !client_has_headroom(clients[i].socket_fd))
        {
            /* Deliberate, and not the same thing as a failure: RC is stepping
             * aside so telemetry keeps its slot. Counted separately so it does
             * not read as packet loss in the logs. */
            clients[i].rc_yielded++;
            continue;
        }

        ssize_t written = write(clients[i].socket_fd, buffer, len);
        if (written < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                /* Client is behind. Drop this frame rather than stall the loop.
                 * Rate-limited so a backed-up client cannot flood the log --
                 * which would itself cost loop time. */
                clients[i].dropped++;
                if ((clients[i].dropped % 100) == 1)
                    LOG_WARN("client %d behind, dropped %lu %s frame(s)",
                             i, (unsigned long)clients[i].dropped, what);
            }
            else if (errno != EINTR && errno != EPIPE)
            {
                LOG_ERROR("Failed to send %s to client %d: %s",
                          what, i, strerror(errno));
            }
        }
        else if ((size_t)written < len)
        {
            /* Should be unreachable: on Linux a non-blocking AF_UNIX
             * SOCK_STREAM write is all-or-nothing for messages that fit in the
             * send buffer -- verified, it returns EAGAIN rather than writing a
             * prefix. If it ever did happen the stream would be desynchronised
             * (a truncated object glued to the next one), so say so loudly
             * rather than counting it as an ordinary drop. */
            LOG_ERROR("PARTIAL WRITE of %s to client %d (%zd/%zu) -- stream framing lost",
                      what, i, written, len);
            clients[i].dropped++;
        }
    }
    pthread_mutex_unlock(&clients_mutex);
}

/* Builders already terminate with "}\n", so this only measures. */
static void ipc_build_and_send(void (*build)(char *, size_t), const char *what,
                               bool low_priority)
{
    char buffer[BUFFER_SIZE];
    buffer[0] = '\0';
    build(buffer, sizeof(buffer) - 1);
    size_t len = strlen(buffer);
    if (len == 0)
        return;
    if (buffer[len - 1] != '\n')
    {
        /* Truncated: the closing "}\n" never fitted. Sending it would hand the
         * reader an unparseable fragment, so drop it and say so. */
        LOG_ERROR("%s packet truncated at %zu bytes, not sent", what, len);
        return;
    }
    ipc_broadcast_line(buffer, len, what, low_priority);
}

/**
 * @brief Broadcast telemetry to all connected clients
 *
 * Call this periodically from the main control loop.
 */
void ipc_broadcast_telemetry(void)
{
    /* Never yields. A missing telemetry sample is a hole in every graph and
     * every recorded CSV. */
    ipc_build_and_send(build_telemetry_json, "telemetry", false);
}

/**
 * @brief Broadcast the RC (SBUS) packet.
 *
 * Small and cheap by design -- call this at the RC rate, not the telemetry
 * rate. See build_rc_json().
 */
void ipc_broadcast_rc(void)
{
    /* Yields when the client is behind -- see TX_RESERVE_BYTES. */
    ipc_build_and_send(build_rc_json, "rc", true);
}

/**
 * @brief Broadcast the config packet (motor / pos / sbus config).
 */
void ipc_broadcast_config(void)
{
    /* Never yields: it is sent once on connect, and a client that misses it
     * shows its controls at built-in defaults until something else changes. */
    ipc_build_and_send(build_config_json, "config", false);
}

/* Worst drop count across connected clients, for the telemetry packet. Exposed
 * so "is the bot dropping packets?" is answerable from the dashboard instead of
 * inferred from the shape of a graph. */
unsigned long ipc_get_tx_drops(void)
{
    unsigned long worst = 0;
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].active && clients[i].dropped > worst)
            worst = clients[i].dropped;
    pthread_mutex_unlock(&clients_mutex);
    return worst;
}

/* Set whenever a command changes something in the config packet. The control
 * loop clears it by calling ipc_broadcast_config_if_dirty() -- config is sent
 * from the loop thread like everything else, so there is exactly one writer per
 * client fd ordering-wise and a command cannot interleave a config packet into
 * the middle of a telemetry packet. */
static volatile sig_atomic_t config_dirty = 0;

void ipc_config_touch(void)
{
    config_dirty = 1;
}

void ipc_broadcast_config_if_dirty(void)
{
    if (!config_dirty)
        return;
    config_dirty = 0;
    ipc_broadcast_config();
}

/**
 * @brief Initialize IPC server
 *
 * Creates Unix domain socket and starts server thread.
 *
 * @return 0 on success, -1 on error
 */
int ipc_server_init(void)
{
    struct sockaddr_un addr;

    LOG_INFO("Initializing IPC server at %s", SOCKET_PATH);

    // Remove old socket file if it exists
    unlink(SOCKET_PATH);

    // Create socket
    server_socket = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_socket < 0)
    {
        LOG_ERROR("Failed to create socket: %s", strerror(errno));
        return -1;
    }

    // Bind socket
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

    if (bind(server_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        LOG_ERROR("Failed to bind socket: %s", strerror(errno));
        close(server_socket);
        return -1;
    }

    // Allow any user (e.g. Node.js running as debian) to connect
    chmod(SOCKET_PATH, 0777);

    // Listen
    if (listen(server_socket, 5) < 0)
    {
        LOG_ERROR("Failed to listen: %s", strerror(errno));
        close(server_socket);
        return -1;
    }

    // Start server thread
    server_running = true;
    if (pthread_create(&server_thread, NULL, server_thread_func, NULL) != 0)
    {
        LOG_ERROR("Failed to create server thread");
        close(server_socket);
        return -1;
    }

    pthread_detach(server_thread);

    LOG_INFO("IPC server started successfully");
    return 0;
}

/**
 * @brief Cleanup IPC server
 *
 * Closes all client connections and server socket.
 */
void ipc_server_cleanup(void)
{
    LOG_INFO("Shutting down IPC server");

    server_running = false;

    // Close server socket (will interrupt accept())
    if (server_socket >= 0)
    {
        close(server_socket);
        server_socket = -1;
    }

    // Close all client connections
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active)
        {
            close(clients[i].socket_fd);
            clients[i].active = false;
        }
    }
    pthread_mutex_unlock(&clients_mutex);

    // Remove socket file
    unlink(SOCKET_PATH);

    LOG_INFO("IPC server stopped");
}