#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# bot_up.sh — bring the bot services up ONE AT A TIME, measuring after each.
#
# Run this ON THE BOT.
#
#   ./tools/bot_up.sh              start each service, pausing to measure
#   ./tools/bot_up.sh --dwell 20   longer sample per stage
#   ./tools/bot_up.sh --status     just show where things stand, change nothing
#
# Why staged: this board runs a 100 Hz control loop, a node websocket bridge, an
# OLED daemon and a battery monitor on ONE 1 GHz core. Starting them together
# and finding the box unusable tells you nothing about which one did it. The
# node bridge alone was measured at 36% of the core.
#
# After each service starts it samples CPU, context switches and load, and
# prints the delta. The stage where the numbers jump is your answer.

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

DWELL=12
STATUS_ONLY=0
for a in "$@"; do
    case "$a" in
        --dwell=*|--dwell) DWELL="${a#--dwell=}" ;;
        --status) STATUS_ONLY=1 ;;
        -h|--help) awk 'NR>3 && /^#/ {sub(/^# ?/,""); print; next} NR>3 {exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
        [0-9]*) DWELL="$a" ;;
        *) echo "unknown option: $a" >&2; exit 1 ;;
    esac
done

# Order matters: the control loop first (it is the point of the machine), then
# the bridge that feeds the dashboard, then the cosmetic and monitoring ones.
SERVICES="balance_bot balance_bot_server bbb_oled batt_monitor"

ctxt()  { awk '/^ctxt /{print $2}' /proc/stat; }
busy()  { awk '/^cpu /{t=0; for(i=2;i<=NF;i++) t+=$i; print t, $5+$6}' /proc/stat; }
load1() { awk '{print $1}' /proc/loadavg; }

measure() {   # measure <seconds> -> "cpu% ctxt/s load"
    local secs="$1" c0 c1 b0 b1
    c0=$(ctxt); b0=$(busy)
    sleep "$secs"
    c1=$(ctxt); b1=$(busy)
    awk -v c0="$c0" -v c1="$c1" -v secs="$secs" -v l="$(load1)" \
        -v t0="${b0% *}" -v i0="${b0#* }" -v t1="${b1% *}" -v i1="${b1#* }" \
        'BEGIN{ dt=t1-t0; di=i1-i0;
                printf "%.0f %.0f %s", dt?100*(dt-di)/dt:0, (c1-c0)/secs, l }'
}

show_status() {
    printf "  %-22s%-12s%-10s\n" "service" "enabled" "active"
    printf "  %s\n" "--------------------------------------------"
    for s in $SERVICES bbot-watch; do
        # `systemctl is-enabled` PRINTS "not-found" and ALSO exits non-zero, so
        # `cmd || echo -` emits both and wrecks the column layout. Take the
        # first line and substitute only when it is genuinely empty.
        en="$(systemctl is-enabled "$s" 2>/dev/null | head -1)"
        ac="$(systemctl is-active  "$s" 2>/dev/null | head -1)"
        printf "  %-22s%-12s%-10s\n" "$s" "${en:--}" "${ac:--}"
    done
}

if [ "$STATUS_ONLY" -eq 1 ]; then
    show_status
    echo
    read -r cpu cs ld <<<"$(measure 5)"
    printf "  now: cpu %s%%   ctxt %s/s   load %s\n" "$cpu" "$cs" "$ld"
    exit 0
fi

echo "==> stopping everything first, so the baseline is real"
sudo systemctl stop $SERVICES 2>/dev/null
sleep 2

echo
printf "  %-24s%8s%10s%8s%10s\n" "after starting" "cpu%" "ctxt/s" "load" "verdict"
printf "  %s\n" "------------------------------------------------------------------"

read -r p_cpu p_cs p_ld <<<"$(measure "$DWELL")"
printf "  %-24s%8s%10s%8s\n" "(nothing running)" "$p_cpu" "$p_cs" "$p_ld"

for s in $SERVICES; do
    if ! systemctl list-unit-files "$s.service" >/dev/null 2>&1; then
        printf "  %-24s%s\n" "$s" "  (no such unit — skipped)"
        continue
    fi
    sudo systemctl start "$s" 2>/dev/null
    sleep 2                       # let it get past its own startup burst
    read -r cpu cs ld <<<"$(measure "$DWELL")"

    d_cpu=$((cpu - p_cpu))
    d_cs=$((cs - p_cs))
    verdict=""
    # A service that adds a third of the core, or 2000 context switches a
    # second, is the one to look at -- on this board that is the difference
    # between a responsive box and one that will not accept ssh.
    [ "$d_cpu" -ge 25 ] && verdict="  <-- +${d_cpu}% cpu"
    [ "$d_cs"  -ge 2000 ] && verdict="$verdict  <-- +${d_cs} ctxt/s"
    if ! systemctl is-active --quiet "$s"; then
        verdict="  <-- FAILED TO START"
    fi
    printf "  %-24s%8s%10s%8s%s\n" "$s" "$cpu" "$cs" "$ld" "$verdict"
    p_cpu=$cpu; p_cs=$cs; p_ld=$ld
done

echo
show_status
echo
echo "  Any stage that jumped is the one to look at. To dial the telemetry"
echo "  rates down without rebuilding, from the dashboard or over the socket:"
echo '      {"type":"set_rates","telemetry":10,"rc":10}'
