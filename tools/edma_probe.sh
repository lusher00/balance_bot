#!/usr/bin/env bash
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# This file is part of balance_bot, licensed under the PolyForm
# Noncommercial License 1.0.0. You may use, study, modify, and share
# it for any noncommercial purpose. Commercial use requires a separate
# license from the author -- contact ryan.lush@gmail.com.
# Full license text: see the LICENSE file in the project root, or
# https://polyformproject.org/licenses/noncommercial/1.0.0/

#
# edma_probe.sh -- does opening and closing the RoboClaw port leak EDMA state?
#
#   sudo ./tools/edma_probe.sh               snapshot only
#   sudo ./tools/edma_probe.sh --cycle 200   bare open/close x200, then diff
#   sudo ./tools/edma_probe.sh --reset 20    run roboclaw_reset.py x20, then diff
#
# --reset is the FAITHFUL reproduction and the one to run first. --cycle is a
# weaker control: it opens and closes the port with no traffic, so there is
# never RX data in flight at close, which is the state most likely to make the
# pause fail.
#
# ---- Why -------------------------------------------------------------------
#
# During the 2026-08-23 read-only event the kernel threw this 3 s before
# balance_bot opened the RoboClaw port:
#
#   WARNING: CPU: 0 PID: 806 at drivers/tty/serial/8250/8250_omap.c:1021
#            omap_8250_rx_dma_flush+0xb9/0xc0
#   Comm: python3
#   omap_8250_rx_dma_flush from omap_8250_shutdown
#   ... from tty_release from __fput from sys_close
#
# A python process closed a serial port and the DMA engine refused to pause
# the UART's RX channel. 33 minutes later mmc0 stopped answering a hardware
# reset and the filesystem went read-only.
#
# That python process was not somebody at a keyboard. It is
# balance_bot.service's own ExecStartPre:
#
#   ExecStartPre=/usr/bin/python3 /home/debian/balance_bot/roboclaw_reset.py
#   ExecStart=/usr/local/bin/balance_bot $BOT_ARGS
#
# which is why the two timestamps are 3.0 s apart. So this teardown runs on
# EVERY service start, and if it is unsound it has been firing for months.
#
# All three MMC controllers and the UARTs are served by one EDMA at
# 0x49000000, so the question is whether a bad UART teardown can poison it.
#
# ---- What the baseline already narrowed -------------------------------------
#
# On a healthy board with the port CLOSED:
#
#   dma0chan2/3    481d8000.mmc  (mmc1/eMMC)  direct
#   dma0chan12/13  47810000.mmc  (mmc2)       via router 44e10f90.dma-router
#   dma0chan22/23  48060000.mmc  (mmc0/SD)    direct
#
# mmc0's channels are direct-mapped and fixed. The AM335x UARTs come off the
# crossbar. So a leaked UART channel can never be re-handed to mmc0, and the
# simple "leaked channel gets reused" story is ruled out.
#
# What is NOT ruled out is controller-level state: the PaRAM slot pool, the
# TPTC transfer queues and the error path are shared across all channels. That
# is a weaker mechanism and it deserves a weaker prior -- but it is cheap to
# test, and this script is the cheap test.
#
# ---- What to look for ------------------------------------------------------
#
#   Channels present after the cycle that were not there before
#       -> a real leak. Each open/close is stranding an EDMA channel, and
#          62 channels is not many. This would be the finding.
#
#   Identical before and after, no new WARN
#       -> teardown is clean in the steady state, and the event needs some
#          other trigger (an in-flight RX at close, a specific baud, a
#          concurrent MMC transfer). Try again with the wheels powered and
#          the RoboClaw actually talking.
#
#   A WARN appears
#       -> reproduced on demand. That alone turns a once-a-week ambush into
#          something debuggable, whatever the MMC connection turns out to be.
#
# Note that WARN_ON_ONCE prints ONCE PER BOOT. If it has already fired this
# boot you will not see it again -- check the count before trusting a quiet
# run, which is what --cycle does.
set -u

# ttyS1, NOT ttyS5. Corrected 2026-08-24.
#
# The WARN in the 2026-08-23 capture came from a python3 process closing a
# serial port -- and that process is roboclaw_reset.py, whose source says
# PORT = '/dev/ttyS1'. So the port whose RX DMA teardown failed is the
# RoboClaw's, not the SBUS receiver's. This script defaulted to ttyS5 and was
# therefore cycling a port that had nothing to do with the warning it was
# built to reproduce. The two are easy to transpose: ttyS5 is the one named on
# the balance_bot command line first (-u), ttyS1 the one after -m.
#
#   ttyS1  48022000.serial  irq 22  -> RoboClaw, 460800 baud   (-m)
#   ttyS5  481aa000.serial  irq 35  -> SBUS receiver, 100000   (-u)
#
# Override with PORT=/dev/ttyS5 to test the other one.
PORT="${PORT:-/dev/ttyS1}"
SUMMARY=/sys/kernel/debug/dmaengine/summary

die() { echo "  error: $*" >&2; exit 1; }

CYCLES=0
MODE=none
case "${1:-}" in
    --cycle) MODE=cycle; CYCLES="${2:-200}" ;;
    --reset) MODE=reset; CYCLES="${2:-20}"  ;;
    "")      MODE=none ;;
    *)       die "unknown option: $1 (expected --cycle N or --reset N)" ;;
esac

RESET_PY=/home/debian/balance_bot/roboclaw_reset.py

[ "$(id -u)" = "0" ] || die "must run as root (sudo) -- debugfs is root-only"
[ -r "$SUMMARY" ] || die "no $SUMMARY
       CONFIG_DEBUG_FS and CONFIG_DMA_ENGINE debugfs are needed, and
       /sys/kernel/debug must be mounted:  sudo mount -t debugfs none /sys/kernel/debug"

snap() { grep -E '^\s+dma[0-9]+chan' "$SUMMARY" | sed 's/^[[:space:]]*//'; }

warn_count() {
    dmesg 2>/dev/null | grep -c "omap_8250_rx_dma_flush" || true
}

echo "  === EDMA channel allocation ==="
cat "$SUMMARY"
echo
echo "  8250 rx_dma_flush warnings so far this boot: $(warn_count)"
echo "  (WARN_ON_ONCE -- a count of 1 means 'at least once', not 'exactly once')"

[ "$MODE" = "none" ] && exit 0

# ---- cycle -----------------------------------------------------------------

# balance_bot owns the port when it is up, and the service also pulls in via
# balance_bot_server.service (Requires=), so stopping the bridge is not enough.
if pgrep -x balance_bot >/dev/null 2>&1; then
    die "balance_bot is running and owns $PORT.
       Stop it first:  sudo systemctl stop balance_bot_server balance_bot
       (stopping the bridge alone will not do it -- balance_bot_server.service
        has Requires=balance_bot.service, so the bot is pulled in either way)
       Opening the port underneath it would fight the control loop."
fi
[ -c "$PORT" ] || die "$PORT is not a character device"
[ "$MODE" = "reset" ] && { [ -r "$RESET_PY" ] || die "not found: $RESET_PY"; }

before="$(snap)"
warn_before="$(warn_count)"

echo
if [ "$MODE" = "reset" ]; then
    echo "  running $(basename "$RESET_PY") x$CYCLES (the real startup path) ..."
    i=0
    while [ "$i" -lt "$CYCLES" ]; do
        # Output suppressed; we are testing the teardown, not the reset.
        /usr/bin/python3 "$RESET_PY" >/dev/null 2>&1
        i=$((i + 1))
        printf '\r    %d/%d' "$i" "$CYCLES"
    done
    printf '\n'
else
    echo "  cycling $PORT bare open/close x$CYCLES ..."
    # Plain shell redirection is enough: opening the tty runs
    # uart_port_startup() (which requests the DMA channels) and closing it runs
    # uart_port_shutdown() -> omap_8250_shutdown() -> omap_8250_rx_dma_flush(),
    # the exact path in the captured stack trace. But with no traffic there is
    # no RX in flight, so this is the weaker of the two tests.
    i=0
    while [ "$i" -lt "$CYCLES" ]; do
        exec 3<>"$PORT" 2>/dev/null || die "cannot open $PORT (in use?)"
        exec 3>&-
        i=$((i + 1))
    done
fi

# Give the driver a moment to finish any deferred teardown before snapshotting.
sleep 2

after="$(snap)"
warn_after="$(warn_count)"

echo
echo "  === after ==="
echo "$after"

echo
if [ "$before" = "$after" ]; then
    echo "  channels: IDENTICAL before and after. No leak from a bare open/close."
else
    echo "  channels: CHANGED. Lines only present afterwards are stranded:"
    diff <(echo "$before") <(echo "$after") | sed 's/^/    /'
fi

echo
if [ "$warn_after" -gt "$warn_before" ]; then
    echo "  *** rx_dma_flush WARN fired during the cycle ($warn_before -> $warn_after)."
    echo "  *** Reproduced. Capture it:  dmesg | grep -A 25 rx_dma_flush"
elif [ "$warn_before" -gt 0 ]; then
    echo "  no NEW warning, but one had already fired this boot, so WARN_ON_ONCE"
    echo "  is spent and this run could not have shown a repeat. Reboot and re-run"
    echo "  before concluding the cycle is clean."
elif [ "$MODE" = "cycle" ]; then
    echo "  no rx_dma_flush warning, and none had fired this boot. This cycle is"
    echo "  genuinely clean -- but a bare open/close is the weak test."
    echo
    echo "  Next:  sudo $0 --reset 20"
    echo "  That runs the real ExecStartPre path, which talks to the RoboClaw"
    echo "  and so has RX data in flight at close."
else
    echo "  no rx_dma_flush warning, and none had fired this boot, over $CYCLES"
    echo "  runs of the exact path that produced it on 2026-08-23."
    echo
    echo "  That is a real negative result. The teardown is sound in isolation,"
    echo "  so if the WARN is genuine it needs a condition this test does not"
    echo "  create -- most likely concurrent MMC traffic on the same EDMA."
    echo "  Try again with the filesystem busy, e.g. in another shell:"
    echo "      while true; do dd if=/dev/urandom of=/tmp/edma_load bs=1M count=8 conv=fsync; done"
    echo "  and if it is still clean after that, this lead is spent -- say so"
    echo "  in HANDOFF.md and stop paying for it."
fi
