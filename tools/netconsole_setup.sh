#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# netconsole_setup.sh — ship the kernel log off the board over UDP, so a
# read-only remount leaves evidence somewhere other than the dead filesystem.
#
#   sudo ./tools/netconsole_setup.sh 192.168.1.50            # one-shot, this boot
#   sudo ./tools/netconsole_setup.sh 192.168.1.50 --persist  # and at every boot
#   sudo ./tools/netconsole_setup.sh 192.168.1.50 --wait 120 # poll for the link first
#   sudo ./tools/netconsole_setup.sh --status                # is it actually armed?
#   sudo ./tools/netconsole_setup.sh --unpersist             # undo --persist
#
# On the receiving machine (the Mac), first:
#
#   nc -u -l 6666 | tee ro_event.log
#
# ── Why ──────────────────────────────────────────────────────────────────────
#
# When ext4 hits an error with errors=remount-ro, the root filesystem goes
# read-only. On this board that also drops the ssh session, and dmesg is no
# longer reachable afterwards — so every method of reading the log AFTER the
# event has already failed by the time you would use it:
#
#   journald   cannot write to a read-only disk; falls back to volatile
#              storage in /run, which is lost at the next reboot
#   dmesg      needs a working shell, and the shell is gone
#   /var/log   read-only, by definition of the failure
#
# The failure destroys the means of observing it. netconsole solves it by being
# none of those things: a kernel module that writes printk straight to a UDP
# socket. No filesystem, no userspace process, no shell.
#
# ── The bug this file used to have — do not reintroduce it ───────────────────
#
# `--persist` wrote `netconsole` into /etc/modules-load.d/. That is loaded by
# systemd-modules-load.service, which runs in early boot — on this board at
# about t=20 s. The wifi dongle's interface does not exist until the rtw_8821au
# firmware loads at about t=57 s. So every boot did this:
#
#   [20.039] netconsole: local port 6665
#   [20.040] netconsole: local IPv4 address 192.168.1.142
#   [20.041] netconsole: interface name 'wlan0'
#   [20.066] netpoll: netconsole: wlan0 doesn't exist, aborting
#   [20.067] netconsole: Not enabling netconsole for cmdline0. Netpoll setup failed
#   [20.079] netconsole: network logging started      <-- LIES
#
# The module initialised successfully with zero live targets and printed
# "network logging started" anyway, so the boot looked fine and nothing was
# ever captured. An entire RO event was missed this way.
#
# Two changes:
#
#   1. --persist now installs a systemd unit ordered after the network, which
#      re-runs this script at boot so the address and MAC are re-resolved
#      rather than baked in. It also polls for the interface (--wait) instead
#      of trusting network-online.target, because with iwd on this box there is
#      no guarantee a wait-online implementation is even enabled.
#   2. Arming is VERIFIED. The configfs path is preferred because writing
#      `enabled` returns a real errno when netpoll setup fails; the module
#      parameter path cannot report failure at all. Either way the script now
#      exits non-zero if the target did not come up, so a boot-time failure
#      shows in `systemctl status netconsole-bot`.
#
# ── Limits, stated honestly ──────────────────────────────────────────────────
#
#   * UDP is unreliable and there is no retransmission. Over wifi, under load,
#     you can lose lines — including, occasionally, the ones you want.
#   * It only captures from the moment it arms. Arm it early.
#   * If the root cause is a power brownout severe enough to reset the SoC, the
#     radio dies with everything else and the last moments are lost. Capturing
#     nothing at all would itself be informative.
#
# For a capture that survives even network death, the bulletproof version is
# the serial console on the Blue's debug header with a USB-TTL adapter. This is
# the version that needs no extra hardware.
set -u

PORT_SRC=6665
PORT_DST=6666
TARGET_NAME=bbot
CFGFS=/sys/kernel/config/netconsole
UNIT=netconsole-bot.service
INSTALLED=/usr/local/sbin/netconsole_setup.sh

die() { echo "  error: $*" >&2; exit 1; }
say() { echo "  $*"; }

# ── argument parsing ────────────────────────────────────────────────────────
TARGET_IP=""
PERSIST=0
UNPERSIST=0
STATUS=0
WAIT_SECS=0

while [ $# -gt 0 ]; do
    case "$1" in
        --persist)    PERSIST=1 ;;
        --unpersist)  UNPERSIST=1 ;;
        --status)     STATUS=1 ;;
        --wait)       shift; WAIT_SECS="${1:-0}" ;;
        --wait=*)     WAIT_SECS="${1#--wait=}" ;;
        -h|--help)    sed -n '6,16p' "$0"; exit 0 ;;
        -*)           die "unknown option: $1" ;;
        *)            TARGET_IP="$1" ;;
    esac
    shift
done

[ "$(id -u)" = "0" ] || die "must run as root (sudo)"

# ── --status: report, do not change anything ────────────────────────────────
#
# "Is netconsole loaded" is the wrong question, and it is the question that let
# the boot-time failure hide. The module can be loaded with no live target.
# What matters is whether a target is ENABLED.
if [ "$STATUS" = "1" ]; then
    if ! lsmod 2>/dev/null | grep -q '^netconsole'; then
        say "netconsole module: NOT loaded"
        exit 1
    fi
    say "netconsole module: loaded"
    armed=0
    if [ -d "$CFGFS" ]; then
        for t in "$CFGFS"/*/; do
            [ -d "$t" ] || continue
            en="$(cat "$t/enabled" 2>/dev/null || echo '?')"
            say "target $(basename "$t"): enabled=$en dev=$(cat "$t/dev_name" 2>/dev/null) $(cat "$t/local_ip" 2>/dev/null) -> $(cat "$t/remote_ip" 2>/dev/null) [$(cat "$t/remote_mac" 2>/dev/null)]"
            [ "$en" = "1" ] && armed=1
        done
    fi
    if [ "$armed" = "0" ]; then
        say "no ENABLED target — loaded but NOT capturing."
        say "(a cmdline/module-parameter target does not appear in configfs;"
        say " if you used that path, check: dmesg | grep -i netpoll)"
        exit 1
    fi
    say "armed. Verify end to end:  echo test | sudo tee /dev/kmsg"
    exit 0
fi

# ── --unpersist ─────────────────────────────────────────────────────────────
if [ "$UNPERSIST" = "1" ]; then
    systemctl disable --now "$UNIT" 2>/dev/null
    rm -f "/etc/systemd/system/$UNIT"
    # The old, broken persistence. Remove it wherever this script has run before.
    rm -f /etc/modules-load.d/netconsole.conf /etc/modprobe.d/netconsole.conf
    systemctl daemon-reload
    say "persistence removed (unit, modules-load.d and modprobe.d)."
    exit 0
fi

[ -n "$TARGET_IP" ] || die "usage: $0 <receiver-ip> [--persist] [--wait SECS]
       $0 --status
       $0 --unpersist"

# ── wait for the link ───────────────────────────────────────────────────────
#
# At boot the dongle's interface appears ~57 s in and DHCP lands later still.
# Poll rather than assume. Zero wait keeps the interactive case instant.
resolve_link() {
    IFACE="$(ip route show default 2>/dev/null | awk '/default/{print $5; exit}')"
    [ -n "${IFACE:-}" ] || return 1
    SRC_IP="$(ip -4 addr show dev "$IFACE" 2>/dev/null | awk '/inet /{sub(/\/.*/,"",$2); print $2; exit}')"
    [ -n "${SRC_IP:-}" ] || return 1
    return 0
}

deadline=$(( $(date +%s) + WAIT_SECS ))
until resolve_link; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
        die "no default route with an IPv4 address$( [ "$WAIT_SECS" -gt 0 ] && echo " after ${WAIT_SECS}s" )"
    fi
    sleep 2
done

# netconsole needs the destination MAC because it bypasses the normal network
# stack — there is no ARP resolution available to it at panic time. Prime the
# neighbour table with a ping, then read it back.
TARGET_MAC="${TARGET_MAC_OVERRIDE:-}"
if [ -z "$TARGET_MAC" ]; then
    say "resolving $TARGET_IP ..."
    mac_deadline=$(( $(date +%s) + WAIT_SECS ))
    while : ; do
        ping -c 2 -W 2 "$TARGET_IP" >/dev/null 2>&1
        TARGET_MAC="$(ip neigh show "$TARGET_IP" 2>/dev/null | awk '/lladdr/{print $5; exit}')"
        [ -n "$TARGET_MAC" ] && break
        [ "$(date +%s)" -lt "$mac_deadline" ] || break
        sleep 2
    done
fi
if [ -z "$TARGET_MAC" ]; then
    die "could not resolve the MAC for $TARGET_IP.
       Is it awake and on the same network? On the Mac: ifconfig en0 | grep ether
       Then pass it by hand:
         sudo TARGET_MAC_OVERRIDE=xx:xx:xx:xx:xx:xx $0 $TARGET_IP"
fi

say "interface : $IFACE ($SRC_IP)"
say "receiver  : $TARGET_IP [$TARGET_MAC] port $PORT_DST"

# ── arm ─────────────────────────────────────────────────────────────────────
#
# Preferred: configfs. `echo 1 > enabled` propagates netpoll_setup()'s errno
# back to the shell, so a failure is a failure. The module-parameter path
# prints "Netpoll setup failed" to the kernel log and returns success anyway,
# which is precisely how this went unnoticed.
arm_configfs() {
    modprobe configfs 2>/dev/null
    mountpoint -q /sys/kernel/config 2>/dev/null || mount -t configfs none /sys/kernel/config 2>/dev/null
    modprobe netconsole 2>/dev/null || return 1
    [ -d "$CFGFS" ] || return 1                      # no CONFIG_NETCONSOLE_DYNAMIC

    t="$CFGFS/$TARGET_NAME"
    if [ -d "$t" ]; then
        echo 0 > "$t/enabled" 2>/dev/null
        rmdir "$t" 2>/dev/null
    fi
    mkdir "$t" 2>/dev/null || return 1

    # Order matters: dev_name and the addresses must be set before enabling.
    echo "$IFACE"      > "$t/dev_name"    || return 1
    echo "$PORT_SRC"   > "$t/local_port"  || return 1
    echo "$SRC_IP"     > "$t/local_ip"    || return 1
    echo "$PORT_DST"   > "$t/remote_port" || return 1
    echo "$TARGET_IP"  > "$t/remote_ip"   || return 1
    echo "$TARGET_MAC" > "$t/remote_mac"  || return 1
    echo 1             > "$t/enabled"     || return 1

    [ "$(cat "$t/enabled" 2>/dev/null)" = "1" ] || return 1
    return 0
}

arm_modparam() {
    cfg="${PORT_SRC}@${SRC_IP}/${IFACE},${PORT_DST}@${TARGET_IP}/${TARGET_MAC}"
    modprobe -r netconsole 2>/dev/null
    modprobe netconsole "netconsole=$cfg" || return 1
    # The module cannot tell us it failed, so read the kernel's own complaint.
    if dmesg 2>/dev/null | tail -40 | grep -q "Netpoll setup failed"; then
        return 1
    fi
    return 0
}

if arm_configfs; then
    say "armed via configfs (target '$TARGET_NAME')."
elif arm_modparam; then
    say "armed via module parameters (configfs target unavailable)."
    say "NOTE: this path cannot verify itself reliably. Run the /dev/kmsg test below."
else
    die "could not arm netconsole.
       Check the module exists:
         ls /lib/modules/\$(uname -r)/kernel/drivers/net/netconsole.ko*
       and the kernel's own account:
         dmesg | grep -iE 'netconsole|netpoll'"
fi

# Make sure everything reaches the console, not just the loud stuff. An ext4
# error is KERN_CRIT and would arrive regardless, but the lines LEADING UP to
# it are the interesting part and some of those are lower priority.
dmesg -n 8 2>/dev/null || true

# ── persist ─────────────────────────────────────────────────────────────────
if [ "$PERSIST" = "1" ]; then
    # Remove the old broken persistence before installing the new one, or the
    # early load races the unit and re-creates the silent-failure case.
    rm -f /etc/modules-load.d/netconsole.conf /etc/modprobe.d/netconsole.conf

    install -m 0755 "$0" "$INSTALLED" || die "could not install to $INSTALLED"

    cat > "/etc/systemd/system/$UNIT" <<EOF
[Unit]
Description=netconsole -> $TARGET_IP:$PORT_DST (kernel log off-board)
Documentation=file://$INSTALLED
# network-online.target is ordering only; with iwd there may be no wait-online
# implementation enabled at all, so the script polls for the link as well.
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
# Re-resolve address and MAC at every boot rather than baking them in: the
# board's own IP changes with DHCP and the receiver's MAC changes whenever the
# Mac moves between wifi and ethernet.
ExecStart=$INSTALLED $TARGET_IP --wait 180
ExecStop=/sbin/modprobe -r netconsole

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable "$UNIT" >/dev/null 2>&1 || die "systemctl enable $UNIT failed"
    say "persisted as $UNIT (runs after the network, re-resolves each boot)."
    say "After the next reboot, confirm with:  systemctl status $UNIT"
    say "                                and:  sudo $0 --status"
fi

echo
say "On the receiver, run this and leave it running:"
say "    nc -u -l $PORT_DST | tee ro_event.log"
echo
say "Verify it works RIGHT NOW, before trusting it with the real event:"
say "    echo 'netconsole test from '\$(hostname) | sudo tee /dev/kmsg"
echo
say "If that line does not appear on the receiver, this is NOT armed —"
say "do not run the workload assuming it is capturing."
