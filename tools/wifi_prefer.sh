#!/bin/sh
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# wifi_prefer.sh — pick the best available network once, at boot.
#
# iwd chooses a network at connect time and then stays on it. It does not roam
# between SSIDs, and it has no priority field: known networks are ranked
# internally by signal and connection history, neither of which is settable. So
# the ordering has to live outside iwd. This is that, and it runs once.
#
# The GL travel router is powered with the Blue, so by the time this runs the
# question is already settled -- either it came up or it is not in use today.
# Nothing here polls: if the router is off, the house APs are the answer and
# they do not stop being the answer ten minutes later.
#
#   ./wifi_prefer.sh            switch if something better is in range
#   ./wifi_prefer.sh -n         say what it would do, change nothing
#
# First entry wins. Anything not on this list is left alone -- if the bot is
# connected to something unlisted and none of these are in range, that
# connection stands.
PREFERRED="TakeFlight_GL50G TakeFlight_GL24G TakeFlight_501 TakeFlight_502"

IFACE="${WIFI_IFACE:-wlan0}"
SCAN_SETTLE=5

DRY=0
[ "$1" = "-n" ] && DRY=1

# iwctl decorates its output with ANSI colour. Everything below reads through this.
plain() { sed 's/\x1b\[[0-9;]*[A-Za-z]//g'; }

current_ssid() {
    iwctl station "$IFACE" show 2>/dev/null | plain |
        sed -n 's/.*Connected network[[:space:]]\{2,\}\(.*[^[:space:]]\)[[:space:]]*$/\1/p'
}

visible_ssids() {
    iwctl station "$IFACE" get-networks 2>/dev/null | plain |
        sed -n 's/^[[:space:]]*>\{0,1\}[[:space:]]*\(.*[^[:space:]]\)[[:space:]]\{2,\}\(psk\|open\|8021x\).*/\1/p'
}

# Rank of an SSID in PREFERRED, or 99 for "not on the list".
rank_of() {
    _r=1
    for _p in $PREFERRED; do
        [ "$1" = "$_p" ] && { echo "$_r"; return; }
        _r=$((_r + 1))
    done
    echo 99
}

cur=$(current_ssid)
cur_rank=$(rank_of "$cur")

# Already on the top choice. Do nothing, and specifically DO NOT SCAN: a scan
# takes the radio off-channel for a few hundred milliseconds per band, which on
# this bot is a stall in the telemetry and the control link.
if [ "$cur_rank" -eq 1 ]; then
    echo "wifi_prefer: already on $cur (first choice)"
    exit 0
fi

iwctl station "$IFACE" scan >/dev/null 2>&1
sleep "$SCAN_SETTLE"
seen=$(visible_ssids)

rank=1
for want in $PREFERRED; do
    # Only ever move UP the list. Reaching the current network's own rank means
    # nothing better is in range, so stay put rather than reconnecting to what
    # we are already associated with.
    [ "$rank" -ge "$cur_rank" ] && break
    if echo "$seen" | grep -qxF "$want"; then
        if [ "$DRY" = 1 ]; then
            echo "wifi_prefer: would switch ${cur:-(none)} -> $want"
            exit 0
        fi
        echo "wifi_prefer: switching ${cur:-(none)} -> $want"
        # connect() drops the current association first, so a missing or wrong
        # credential leaves the bot with no link at all. Say so loudly rather
        # than exiting 0 on a silent failure.
        if iwctl station "$IFACE" connect "$want" >/dev/null 2>&1; then
            sleep 3
            echo "wifi_prefer: now on $(current_ssid)"
            exit 0
        fi
        echo "wifi_prefer: connect to $want FAILED — check the stored credential:" >&2
        echo "    iwctl known-networks \"$want\" show" >&2
        exit 1
    fi
    rank=$((rank + 1))
done

echo "wifi_prefer: nothing better in range, staying on ${cur:-(nothing)}"
exit 0
