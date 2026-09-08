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
# consoles.sh — serial console helpers for the boards on this desk.
#
# Source it from ~/.zshrc:
#
#     source ~/balance_bot/tools/consoles.sh
#
# Then:
#
#     bone-console         minicom on boneblue-0 at 921600
#     pi5-console          minicom on pi5-0 at 115200
#     consoles             which cables are plugged in right now
#     consoles install     write ~/.minirc.<name> so `minicom <name>` works too
#
# ---- two macOS specifics this exists to get right ------------------------
#
# LOOKED UP BY SERIAL, NOT BY PATH. macOS builds the device name from the
# serial number burned into the FTDI EEPROM, so the cable keeps its identity
# across ports, reboots and machines. Matching on the serial rather than
# hardcoding the whole path also survives Apple changing the prefix, which it
# has done before (usbserial- vs usbmodem-).
#
# cu.*, NOT tty.*. Opening /dev/tty.* blocks until the port raises carrier
# detect. A three-wire console cable never raises it, so minicom just hangs
# with no error at all. /dev/cu.* -- "call-out" -- is the node for talking TO
# a device, and it does not wait.
#
# Hardware flow control is forced off for the same class of reason: with no RTS
# and CTS wired, minicom will happily connect and then silently refuse to send
# anything you type.

# name : FTDI serial : baud
#
# boneblue-0 appears twice on purpose, same cable at two speeds. U-Boot on the
# BeagleBone talks at its built-in 115200 and knows nothing about the kernel
# cmdline, so once Linux takes the console over at 921600 the same connection
# turns to garbage — and vice versa, which is why watching a boot at 921600
# shows nothing until the handover. Pick the speed for the stage you want to
# be in: bone-boot-console to catch U-Boot and interrupt autoboot,
# bone-console for the running system.
BB_CONSOLES=(
    "boneblue-0:FTCHJJ13:921600"
    "bone-boot-console:FTCHJJ13:115200"
    "pi5-0:FT9J0YQH:115200"
)

# Overridable so the lookup can be tested without real hardware.
: "${BB_CONSOLE_DEV_DIR:=/dev}"

# Resolve an FTDI serial to a call-out device path, or fail.
#
# Deliberately not a glob: an unmatched glob is an error in zsh and a literal
# string in bash, and this file is sourced into an interactive zsh.
_bb_console_dev() {
    local serial="$1" dev
    for dev in $(/bin/ls "$BB_CONSOLE_DEV_DIR" 2>/dev/null); do
        case "$dev" in
            cu.*"$serial"*) printf '%s/%s\n' "$BB_CONSOLE_DEV_DIR" "$dev"; return 0 ;;
        esac
    done
    return 1
}

_bb_console_list() {
    local entry name serial baud dev
    printf '  %-18s %-10s %-8s %s\n' NAME SERIAL BAUD DEVICE
    for entry in "${BB_CONSOLES[@]}"; do
        name="${entry%%:*}"
        serial="${entry#*:}"; serial="${serial%%:*}"
        baud="${entry##*:}"
        if dev="$(_bb_console_dev "$serial")"; then
            printf '  %-18s %-10s %-8s %s\n' "$name" "$serial" "$baud" "$dev"
        else
            printf '  %-18s %-10s %-8s %s\n' "$name" "$serial" "$baud" "-- not plugged in --"
        fi
    done
}

_bb_console_open() {
    local name="$1" serial="$2" baud="$3"; shift 3
    local dev

    if ! command -v minicom >/dev/null 2>&1; then
        printf 'minicom is not installed. brew install minicom\n' >&2
        return 1
    fi

    if ! dev="$(_bb_console_dev "$serial")"; then
        printf '%s: cable %s is not plugged in.\n\n' "$name" "$serial" >&2
        _bb_console_list >&2
        return 1
    fi

    printf '%s  %s  %s 8N1   (Ctrl-A X to quit, Ctrl-A Z for help)\n' \
        "$name" "$dev" "$baud"

    # --noinit skips the modem init string; there is no modem.
    minicom --device "$dev" --baudrate "$baud" --noinit --color=on "$@"
}

# Write a minicom profile per board, so `minicom boneblue-0` also works —
# handy when something else wants to launch minicom for you.
#
# The concrete device path gets baked in here, so re-run this if a cable ever
# comes up under a different name. The shell functions above resolve it live
# and never go stale.
_bb_console_install() {
    local entry name serial baud dev rc written=0
    for entry in "${BB_CONSOLES[@]}"; do
        name="${entry%%:*}"
        serial="${entry#*:}"; serial="${serial%%:*}"
        baud="${entry##*:}"
        if ! dev="$(_bb_console_dev "$serial")"; then
            printf 'skipping %s — cable %s not plugged in\n' "$name" "$serial" >&2
            continue
        fi
        rc="$HOME/.minirc.$name"
        cat > "$rc" <<RC
# Written by balance_bot/tools/consoles.sh — re-run 'consoles install' to update.
# Use with: minicom $name
pu port             $dev
pu baudrate         $baud
pu bits             8
pu parity           N
pu stopbits         1
pu rtscts           No
pu xonxoff          No
RC
        printf 'wrote %s  ->  %s @ %s\n' "$rc" "$dev" "$baud"
        written=$((written + 1))
    done
    [ "$written" -gt 0 ] || return 1
}

bone-console()      { _bb_console_open boneblue-0        FTCHJJ13 921600 "$@"; }
bone-boot-console() { _bb_console_open bone-boot-console FTCHJJ13 115200 "$@"; }
pi5-console()       { _bb_console_open pi5-0             FT9J0YQH 115200 "$@"; }

consoles() {
    case "${1:-status}" in
        status)  _bb_console_list ;;
        install) _bb_console_install ;;
        *)       printf 'usage: consoles [status|install]\n' >&2; return 1 ;;
    esac
}

# Run directly (./tools/consoles.sh install) rather than sourced: do the thing
# and exit.
#
# Matching on the known verbs rather than "is $1 set" matters in zsh, where a
# bare `source file` hands the sourced script the *caller's* positional
# parameters. Anything else is ignored, so sourcing can never trip this.
case "${1:-}" in
    status|install) consoles "$1" ;;
esac
