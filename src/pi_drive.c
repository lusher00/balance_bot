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
 * @file pi_drive.c
 * @brief Drive commands proposed by the Raspberry Pi over robot-link.
 *
 * The Pi (hailo-tracker's follower) sends {"type":"drive","x":..,"y":..,
 * "ttl_ms":..} about ten times a second; robot-link-boned relays each one to
 * the IPC socket. x steers (-1..1, + = right), y drives (-1..1, + = forward),
 * the same normalised units as sbus_get_turn() / sbus_get_drive().
 *
 * The Pi only proposes. Whether a command is used is decided every tick in
 * robot.c, and the answer is published here as a state for telemetry, so the
 * Pi can show whether the Bone is actually obeying it:
 *
 *   disarmed      not armed -- nothing drives, and the gate is shut
 *   gate closed   the dashboard's Pi-drive gate is off (the default)
 *   rc kill       a transmitter is on and its kill switch is not at RUN
 *   stick         the SBUS stick is off centre: the operator wins
 *   no command    gate open, but no unexpired command from the Pi
 *   applying      the Pi's x/y are the drive input this tick
 *
 * The gate starts closed, and robot.c closes it whenever the robot is not
 * armed (disarm, e-stop, boot), so it always takes a deliberate act to hand
 * the robot to the Pi. A command that is not refreshed within its ttl_ms is
 * treated exactly like a centred stick.
 */

#include "debug_config.h"
#include "balance_bot.h"
#include <math.h>
#include <pthread.h>

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static float g_x = 0.0f, g_y = 0.0f;
static uint64_t g_expires_ns = 0;
static volatile int g_gate = 0;
static volatile pi_drive_state_t g_state = PI_DRIVE_DISARMED;

static float clamp1(float v)
{
    if (!isfinite(v))
        return 0.0f;
    return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
}

void pi_drive_set(float x, float y, int ttl_ms)
{
    if (ttl_ms < PI_DRIVE_TTL_MIN_MS)
        ttl_ms = PI_DRIVE_TTL_MIN_MS;
    if (ttl_ms > PI_DRIVE_TTL_MAX_MS)
        ttl_ms = PI_DRIVE_TTL_MAX_MS;
    pthread_mutex_lock(&g_mutex);
    g_x = clamp1(x);
    g_y = clamp1(y);
    g_expires_ns = rc_nanos_since_boot() + (uint64_t)ttl_ms * 1000000ULL;
    pthread_mutex_unlock(&g_mutex);
}

int pi_drive_get(float *x, float *y)
{
    int fresh;
    pthread_mutex_lock(&g_mutex);
    fresh = rc_nanos_since_boot() < g_expires_ns;
    *x = fresh ? g_x : 0.0f;
    *y = fresh ? g_y : 0.0f;
    pthread_mutex_unlock(&g_mutex);
    return fresh;
}

void pi_drive_set_gate(int open)
{
    if (open != g_gate)
    {
        g_gate = open ? 1 : 0;
        LOG_WARN("Pi drive gate %s", g_gate ? "OPEN" : "closed");
    }
}

int pi_drive_gate(void) { return g_gate; }

void pi_drive_set_state(pi_drive_state_t s) { g_state = s; }

pi_drive_state_t pi_drive_state(void) { return g_state; }

pi_drive_state_t pi_drive_decide(int armed, int gate, int sbus_connected,
                                 int sbus_kill, float stick_drive,
                                 float stick_turn, int pi_fresh)
{
    /* Order matters: every earlier reason outranks the later ones. */
    if (!armed)
        return PI_DRIVE_DISARMED;
    if (!gate)
        return PI_DRIVE_GATE_CLOSED;
    if (sbus_connected && sbus_kill < 2)
        return PI_DRIVE_RC_KILL;
    if (stick_drive != 0.0f || stick_turn != 0.0f)
        return PI_DRIVE_STICK;    /* input_sbus.c has already deadbanded */
    if (!pi_fresh)
        return PI_DRIVE_NO_COMMAND; /* expired = centred stick */
    return PI_DRIVE_APPLYING;
}

const char *pi_drive_state_name(pi_drive_state_t s)
{
    switch (s)
    {
    case PI_DRIVE_DISARMED:    return "disarmed";
    case PI_DRIVE_GATE_CLOSED: return "gate closed";
    case PI_DRIVE_RC_KILL:     return "rc kill";
    case PI_DRIVE_STICK:       return "stick";
    case PI_DRIVE_NO_COMMAND:  return "no command";
    case PI_DRIVE_APPLYING:    return "applying";
    }
    return "?";
}
