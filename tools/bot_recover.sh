#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# bot_recover.sh — get back into a bot that is too busy to accept ssh.
#
# Run this ON THE MAC, then power-cycle the bot. It hammers ssh until one
# connection lands and immediately stops the heavy services, before load builds
# far enough to lock you out again.
#
#   ./tools/bot_recover.sh              stop balance_bot + the node bridge
#   ./tools/bot_recover.sh --disable    also disable them, so a reboot stays quiet
#   ./tools/bot_recover.sh --all        stop every bot service including the OLED
#
# Why this is needed: the services start at boot and saturate a single core
# within seconds. There is a window after power-on where sshd is up and the
# load has not arrived yet, and it is too short to catch by hand.

# ── refuse to be sourced ──────────────────────────────────────────────────
# This script calls exit() on error. When a script is SOURCED, exit terminates
# the calling shell -- over ssh that drops the connection, which is a hard way
# to find out you typed `source` instead of `./`. It also breaks $0: sourced,
# $0 is "-bash", so `dirname "$0"` fails with "invalid option -- b".
#
# `(return 0 2>/dev/null)` succeeds only inside a sourced file, which is the
# portable way to detect it.
if (return 0 2>/dev/null); then
    printf 'This is a script, not something to source. Run it:\n    %s%s\n' \
        "" "${BASH_SOURCE[0]}" >&2
    return 1
fi

set -u

HOST="${BOT_HOST:-boneblue-0}"
DISABLE=0
SERVICES="balance_bot balance_bot_server"

for a in "$@"; do
    case "$a" in
        --disable) DISABLE=1 ;;
        --all)     SERVICES="balance_bot balance_bot_server bbb_oled batt_monitor bbot-watch" ;;
        --host=*)  HOST="${a#--host=}" ;;
        -h|--help) awk 'NR>3 && /^#/ {sub(/^# ?/,""); print; next} NR>3 {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option: $a" >&2; exit 1 ;;
    esac
done

ACTION="stop"
[ "$DISABLE" -eq 1 ] && ACTION="disable --now"

echo "==> target: $HOST"
echo "==> will run: sudo systemctl $ACTION $SERVICES"
echo "==> power-cycle the bot now; this keeps trying until it gets in"
echo

n=0
start=$(date +%s)
while true; do
    n=$((n + 1))
    # -o BatchMode=yes so a password prompt cannot stall the loop forever.
    # Short timeouts: we want many fast attempts, not few patient ones -- the
    # window we are trying to hit is a couple of seconds wide.
    if ssh -o ConnectTimeout=3 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
           "$HOST" "sudo systemctl $ACTION $SERVICES" 2>/dev/null; then
        el=$(( $(date +%s) - start ))
        echo
        echo "==> in after $n attempt(s), ${el}s — services $ACTION"
        echo
        ssh -o ConnectTimeout=5 "$HOST" \
            'uptime; echo; systemctl is-active balance_bot balance_bot_server 2>&1 | paste -sd" "' 2>/dev/null
        echo
        echo "==> the board should be responsive now. Bring things back with:"
        echo "      ./tools/bot_up.sh"
        exit 0
    fi
    printf '\r    attempt %d ...' "$n"
    sleep 1
done
