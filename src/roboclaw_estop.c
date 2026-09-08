// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
//
// This file is part of balance_bot, licensed under the PolyForm
// Noncommercial License 1.0.0. You may use, study, modify, and share
// it for any noncommercial purpose. Commercial use requires a separate
// license from the author -- contact ryan.lush@gmail.com.
// Full license text: see the LICENSE file in the project root, or
// https://polyformproject.org/licenses/noncommercial/1.0.0/

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "balance_bot.h"
#include "debug_config.h"
#include "roboclaw_estop.h"

/*
 * The RoboClaw e-stop line is AM335x GPIO1_25 — the pin legacy sysfs numbering
 * called GPIO 57.
 *
 * This file used to hardcode 537, which is 480 + 57: correct only while
 * gpiochip0 was based at 480. Kernels since then base it at 512, which puts the
 * bank-1 chip at 544 and the real pin at 569. The stale constant still resolves
 * to a valid node — gpio0_25, an unrelated pin — so asserting and clearing the
 * e-stop drove the wrong line, succeeded, and logged nothing. The failure was
 * invisible from the dashboard and from the journal.
 *
 * Two lessons are baked in below:
 *   1. Never hardcode a sysfs GPIO number. Find the chip that owns the
 *      controller and add the bank offset. That survives the next base change.
 *   2. Never discard the result of a GPIO write. A safety interlock that fails
 *      quietly is worse than one that isn't there, because you trust it.
 *
 * Longer term this should move to libgpiod, which addresses lines by
 * (chip, offset) natively and doesn't have a global numbering space at all.
 * Kept on sysfs for now to hold the change small.
 */

/*
 * Match on the controller's MMIO address ONLY.
 *
 * Do not be tempted to match the gpiochip label. Labels like "gpio-0-31" are
 * assigned in *registration* order, not by hardware bank, and on this board
 * GPIO0 registers last because it hangs off a different interconnect:
 *
 *     gpiochip512  label gpio-0-31     4804c000.gpio  = GPIO1
 *     gpiochip544  label gpio-32-63    481ac000.gpio  = GPIO2
 *     gpiochip576  label gpio-64-95    481ae000.gpio  = GPIO3
 *     gpiochip608  label gpio-96-127   44e07000.gpio  = GPIO0
 *
 * So "gpio-32-63" is GPIO2, not GPIO1, and trusting it lands you on the wrong
 * pin. The MMIO address is fixed by the SoC and is the only durable key.
 */
#define ESTOP_BANK_ADDR   "4804c000"   /* AM335x GPIO1 controller MMIO base */
#define ESTOP_BANK_OFFSET 25           /* GPIO1_25 — legacy sysfs number 57 */

/* Overridable so the resolver can be tested against a fake sysfs tree. */
#ifndef SYSFS_GPIO
#define SYSFS_GPIO "/sys/class/gpio"
#endif

static int g_estop_gpio = -1; /* resolved sysfs number; -1 = unresolved */
static int g_warned = 0;      /* so a broken pin logs once, not at loop rate */

/* ── small sysfs helpers ─────────────────────────────────────────── */

static int read_text(const char *path, char *buf, size_t n)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0)
        return -1;
    ssize_t r = read(fd, buf, n - 1);
    close(fd);
    if (r < 0)
        return -1;
    buf[r] = '\0';
    while (r > 0 && (buf[r - 1] == '\n' || buf[r - 1] == ' '))
        buf[--r] = '\0';
    return 0;
}

static int write_text(const char *path, const char *val)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0)
        return -1;
    size_t len = strlen(val);
    ssize_t w = write(fd, val, len);
    int err = errno;
    close(fd);
    if (w != (ssize_t)len)
    {
        errno = err;
        return -1;
    }
    return 0;
}

/*
 * Paths are built with a bounded "%.*s" for the chip name. struct dirent's
 * d_name can be 255 bytes, which GCC (correctly) points out could overrun a
 * 256-byte buffer once chip_base() is inlined. Real gpiochip names are ~11
 * characters, so clamping at CHIP_NAME_MAX loses nothing and keeps the bound
 * provable at compile time.
 */
#define CHIP_NAME_MAX 63
#define GPIO_PATH_MAX 512

static int chip_base(const char *chip)
{
    char path[GPIO_PATH_MAX], buf[32];
    snprintf(path, sizeof path, "%s/%.*s/base", SYSFS_GPIO, CHIP_NAME_MAX, chip);
    if (read_text(path, buf, sizeof buf) != 0)
        return -1;
    return atoi(buf);
}

/*
 * Find the sysfs number of GPIO1_25 on this kernel.
 *
 * Primary strategy: every gpiochip has a "device" symlink naming the platform
 * device that owns it (…/4804c000.gpio). That address is fixed by the SoC and
 * cannot drift.
 *
 * Fallback: match the chip label. Less durable — the label format has changed
 * across kernel versions — but it costs nothing and covers the case where the
 * device symlink is missing.
 */
static int resolve_estop_gpio(void)
{
    DIR *d = opendir(SYSFS_GPIO);
    if (!d)
    {
        LOG_ERROR("roboclaw_estop: cannot open %s (%s) — is sysfs GPIO enabled?",
                  SYSFS_GPIO, strerror(errno));
        return -1;
    }

    int base = -1;
    struct dirent *e;
    while ((e = readdir(d)) != NULL)
    {
        if (strncmp(e->d_name, "gpiochip", 8) != 0)
            continue;

        char path[GPIO_PATH_MAX], link[GPIO_PATH_MAX];

        /* readlink() gives a relative path; that is fine, the controller
         * address appears in the final component either way. */
        snprintf(path, sizeof path, "%s/%.*s/device", SYSFS_GPIO, CHIP_NAME_MAX, e->d_name);
        ssize_t n = readlink(path, link, sizeof link - 1);
        if (n <= 0)
            continue;
        link[n] = '\0';
        if (!strstr(link, ESTOP_BANK_ADDR))
            continue;

        int b = chip_base(e->d_name);
        if (b < 0)
            continue;
        if (base >= 0 && base != b)
            LOG_WARN("roboclaw_estop: more than one gpiochip claims %s "
                     "(base %d and %d) — using %d",
                     ESTOP_BANK_ADDR, base, b, b);
        base = b;
    }
    closedir(d);

    if (base < 0)
    {
        LOG_ERROR("roboclaw_estop: no gpiochip owns %s.gpio (AM335x GPIO1) "
                  "— E-STOP IS INOPERATIVE",
                  ESTOP_BANK_ADDR);
        return -1;
    }
    LOG_INFO("roboclaw_estop: %s.gpio is gpiochip base %d", ESTOP_BANK_ADDR, base);
    return base + ESTOP_BANK_OFFSET;
}

/* ── public API ──────────────────────────────────────────────────── */

int roboclaw_estop_init(void)
{
    g_estop_gpio = resolve_estop_gpio();
    if (g_estop_gpio < 0)
        return -1;

    char node[256], dirp[256], valp[256], num[16];
    snprintf(num, sizeof num, "%d", g_estop_gpio);
    snprintf(node, sizeof node, SYSFS_GPIO "/gpio%d", g_estop_gpio);
    snprintf(dirp, sizeof dirp, SYSFS_GPIO "/gpio%d/direction", g_estop_gpio);
    snprintf(valp, sizeof valp, SYSFS_GPIO "/gpio%d/value", g_estop_gpio);

    if (access(node, F_OK) != 0)
    {
        if (write_text(SYSFS_GPIO "/export", num) != 0)
        {
            LOG_ERROR("roboclaw_estop: export of gpio%d failed (%s) — E-STOP IS INOPERATIVE",
                      g_estop_gpio, strerror(errno));
            g_estop_gpio = -1;
            return -1;
        }
        usleep(50000); /* udev must create the node and fix its permissions */
    }

    /*
     * Write "high", not "out".
     *
     * Writing "out" configures the line as an output driven LOW, and only the
     * subsequent value write brings it back up. That falling edge re-latches
     * the RoboClaw. It matters because init runs three times during startup
     * (roboclaw_init, motor_hal_init, robot_init), all of them AFTER
     * roboclaw_reset.py has already brought the unit up clean — so the e-stop
     * could always be cleared by hand (a bare value write, no edge) but never
     * survived a boot.
     *
     * "high" configures output-already-high atomically, with no glitch.
     */
    if (write_text(dirp, "high") != 0)
    {
        LOG_WARN("roboclaw_estop: gpio%d direction=high rejected (%s) — falling back "
                 "to out+value, which glitches the line low briefly",
                 g_estop_gpio, strerror(errno));
        if (write_text(dirp, "out") != 0 || write_text(valp, "1") != 0)
        {
            LOG_ERROR("roboclaw_estop: cannot configure gpio%d (%s) — E-STOP IS INOPERATIVE",
                      g_estop_gpio, strerror(errno));
            g_estop_gpio = -1;
            return -1;
        }
    }

    int lvl = roboclaw_estop_get();
    if (lvl != 1)
    {
        LOG_ERROR("roboclaw_estop: gpio%d reads %d after init, expected 1 — "
                  "the line is not deasserted",
                  g_estop_gpio, lvl);
        return -1;
    }

    LOG_INFO("roboclaw_estop: GPIO1_%d -> sysfs gpio%d (deasserted, no glitch)",
             ESTOP_BANK_OFFSET, g_estop_gpio);
    return 0;
}

static void estop_set(int level, const char *what)
{
    if (g_estop_gpio < 0)
    {
        if (!g_warned)
        {
            LOG_ERROR("roboclaw_estop: %s ignored — GPIO never resolved. "
                      "The e-stop line is NOT being driven.",
                      what);
            g_warned = 1;
        }
        return;
    }
    char path[256];
    snprintf(path, sizeof path, SYSFS_GPIO "/gpio%d/value", g_estop_gpio);
    if (write_text(path, level ? "1" : "0") != 0)
        LOG_ERROR("roboclaw_estop: %s failed writing %s (%s)",
                  what, path, strerror(errno));
}

void roboclaw_estop_assert(void) { estop_set(0, "assert"); }

void roboclaw_estop_deassert(void) { estop_set(1, "deassert"); }

int roboclaw_estop_get(void)
{
    if (g_estop_gpio < 0)
        return -1;
    char path[256], buf[8];
    snprintf(path, sizeof path, SYSFS_GPIO "/gpio%d/value", g_estop_gpio);
    if (read_text(path, buf, sizeof buf) != 0)
        return -1;
    return buf[0] == '1' ? 1 : 0;
}

int roboclaw_estop_gpio(void) { return g_estop_gpio; }
