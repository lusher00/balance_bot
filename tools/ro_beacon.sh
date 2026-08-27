#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# ro_beacon.sh -- a heartbeat that survives the read-only fault, so the next
#                 event arrives with a trend attached instead of a wreck.
#
#   sudo ./tools/ro_beacon.sh --stdout        watch it locally first
#   sudo ./tools/ro_beacon.sh                 run in the foreground, to kmsg
#   sudo ./tools/ro_beacon.sh --install       systemd unit, enabled at boot
#   sudo ./tools/ro_beacon.sh --uninstall     remove it
#
# ---- Why -------------------------------------------------------------------
#
# The 2026-08-23 capture (ro_dump.txt) is the only primary evidence this fault
# has ever produced, and its defining feature is a hole: mmc0 enumerates at
# t=4.4 s and then the kernel says NOTHING about it for 59 minutes, at which
# point the card fails a hardware reset and the filesystem is gone. There is no
# trend to read because nothing was sampling anything.
#
# The same capture also proved, by accident, that a sampler WOULD have
# survived. bbb_oled's broken probe hit the i2c core every 30 s and its
# rejection was logged every time: 189 lines, 174 steady-state intervals,
# min 30.080 s, max 30.336 s -- a 256 ms spread across 91 minutes, with the
# intervals straddling both storage deaths measuring 30.203 s and 30.179 s,
# i.e. indistinguishable from the rest. A userspace process on a 30 s timer
# kept perfect time through the whole event. Nothing about this fault stops a
# beacon from running; there was simply never one to run.
#
# So: one line every 10 s, from virtual filesystems only (/proc, /sys, /run --
# all RAM), to /dev/kmsg. It touches no block device, so it cannot be blocked
# by the dead one, and with tools/netconsole_setup.sh armed it leaves the board
# entirely.
#
# ---- What each field is for ------------------------------------------------
#
#   v=       battery volts from /run/batt_status.json (batt_monitor writes it).
#            THIS IS THE FIELD THE POWER THEORY HAS BEEN MISSING. Months of
#            arguing power and there has never once been a voltage trace at the
#            moment of the failure. If the rail sags before mmc0 dies, this is
#            what shows it; if it is flat through the event, that is just as
#            useful and considerably cheaper than another hardware swap.
#   dma=     in-use/total EDMA channels. If the shared-engine theory is right,
#            in-use should climb over hours. If it is flat, the leak story dies
#            and only controller-level state is left (see edma_probe.sh).
#   mmc0=    presence of the card's device directory. Anything other than 'ok'
#            before the I/O errors would be the first degradation signal this
#            fault has ever shown.
#   irq deltas  per-interval interrupt counts for mmc0 and i2c1. A storm or a
#            stall on either shows up here before the block layer notices.
#   root=    rw or ro. The transition is what we are here for.
#
# On the transition it dumps the EDMA summary and the mmc0 state to kmsg ONCE.
# That snapshot -- taken at the moment of the fault rather than reconstructed
# 30 minutes later from a shell that can barely exec -- is the single piece of
# evidence every previous attempt has failed to collect.
#
# ---- Gotchas that are already handled --------------------------------------
#
# - Lines go out at <4> (KERN_WARNING). Plain text written to /dev/kmsg lands
#   at KERN_DEFAULT, which on this board sits below console_loglevel, so it
#   would be recorded but NOT forwarded to a console -- and netconsole is a
#   console. A beacon that never leaves the board is the netconsole mistake
#   again with extra steps.
# - The transition line is emitted three times. Losing the one packet that
#   matters to a UDP drop, during the exact seconds the machine is falling
#   over, is not a risk worth taking to save two lines.
# - No subprocess per field. One awk per tick on a 1 GHz single core, at 10 s
#   cadence, is ~0.1% CPU; a dozen greps would not be.
set -u

INTERVAL="${INTERVAL:-10}"
KMSG=/dev/kmsg
BATT=/run/batt_status.json
SUMMARY=/sys/kernel/debug/dmaengine/summary
UNIT=/etc/systemd/system/ro_beacon.service
SELF_INSTALLED=/usr/local/bin/ro_beacon.sh

MODE=run
case "${1:-}" in
    --stdout)    MODE=stdout ;;
    --install)   MODE=install ;;
    --uninstall) MODE=uninstall ;;
    "")          MODE=run ;;
    *) echo "  unknown option: $1 (--stdout | --install | --uninstall)" >&2; exit 1 ;;
esac

die() { echo "  error: $*" >&2; exit 1; }

# ---- install / uninstall ---------------------------------------------------

if [ "$MODE" = install ] || [ "$MODE" = uninstall ]; then
    [ "$(id -u)" = 0 ] || die "must run as root (sudo)"
fi

if [ "$MODE" = uninstall ]; then
    /usr/bin/systemctl disable --now ro_beacon.service 2>/dev/null
    rm -f "$UNIT" "$SELF_INSTALLED"
    /usr/bin/systemctl daemon-reload
    echo "  removed $UNIT and $SELF_INSTALLED"
    exit 0
fi

if [ "$MODE" = install ]; then
    src="$(readlink -f "$0")"
    install -m 0755 "$src" "$SELF_INSTALLED" || die "cannot install to $SELF_INSTALLED"
    cat > "$UNIT" <<EOF
[Unit]
Description=read-only fault beacon (kmsg heartbeat)
# Wants, not Requires: the beacon is more useful than the thing it watches, so
# it must never be held up or pulled down by another unit.
After=batt_monitor.service
Wants=batt_monitor.service

[Service]
Type=simple
ExecStart=$SELF_INSTALLED
Restart=always
RestartSec=5
# It reads /proc, /sys/kernel/debug and /run and writes /dev/kmsg. Nothing
# else. ProtectSystem=strict is safe here and keeps it from ever being the
# process that dirties a page on the card we are trying to observe.
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes

[Install]
WantedBy=multi-user.target
EOF
    /usr/bin/systemctl daemon-reload
    /usr/bin/systemctl enable --now ro_beacon.service || die "failed to start"
    echo "  installed and started. Verify it is really emitting:"
    echo "      journalctl -k -n 5 | grep ro_beacon"
    echo "  and that it leaves the board:"
    echo "      sudo ./tools/netconsole_setup.sh --status"
    exit 0
fi

# ---- run -------------------------------------------------------------------

if [ "$MODE" = run ]; then
    [ "$(id -u)" = 0 ] || die "must run as root (sudo) -- /dev/kmsg is root-write"
    [ -w "$KMSG" ] || die "cannot write $KMSG"
fi

emit() {
    if [ "$MODE" = stdout ]; then
        printf '%s\n' "$1"
    else
        printf '<4>%s\n' "$1" > "$KMSG" 2>/dev/null
    fi
}

# Battery volts, or "-". The file is tmpfs and tiny; a read costs nothing.
batt_v() {
    [ -r "$BATT" ] || { printf -- '-'; return; }
    # One awk, no subshell chain. Accepts voltage / voltage_v / v.
    /usr/bin/awk 'match($0,/"(voltage|voltage_v|v)"[[:space:]]*:[[:space:]]*-?[0-9.]+/){
        s=substr($0,RSTART,RLENGTH); sub(/.*:[[:space:]]*/,"",s); printf "%s",s; found=1; exit
    } END{ if(!found) printf "-" }' "$BATT" 2>/dev/null || printf -- '-'
}

# in-use/total EDMA channels. Format is "dma0chan12   | 47810000.mmc:rx", so a
# channel with text after the pipe has a client.
dma_state() {
    [ -r "$SUMMARY" ] || { printf -- '-/-'; return; }
    /usr/bin/awk '/chan[0-9]+/{ total++; if ($0 ~ /\|[[:space:]]*[^[:space:]]/) used++ }
                  END{ printf "%d/%d", used+0, total+0 }' "$SUMMARY" 2>/dev/null || printf -- '-/-'
}

# Cumulative interrupt counts, one pass. Note mmc0/mmc1/mmc2 appear in
# /proc/interrupts under those bare names, not their MMIO addresses.
#
# dma_ccerrint is the EDMA channel-controller ERROR interrupt, and it is the
# most interesting counter on this board: the 2026-08-23 capture showed it
# sitting at 43. Not zero. There is no baseline for it yet -- it may all be
# from boot -- so what matters is whether it MOVES at the moment mmc0 dies.
# If it steps at the failure, EDMA errors are directly implicated and the
# theory is proven. If it never moves again, the theory is finished.
irq_counts() {
    /usr/bin/awk '{
        n=0; for (i=2; i<=NF; i++) if ($i ~ /^[0-9]+$/) n+=$i; else break
        if ($NF == "mmc0")               mmc0+=n
        if ($NF == "mmc1")               mmc1+=n
        if ($0 ~ /4802a000\.i2c/)        i2c+=n
        if ($0 ~ /dma_ccerrint/)         ccerr+=n
        if ($0 ~ /dma_ccint/)            ccint+=n
    } END { printf "%d %d %d %d %d", mmc0+0, mmc1+0, i2c+0, ccerr+0, ccint+0 }' \
        /proc/interrupts 2>/dev/null || printf '0 0 0 0 0'
}

root_mode() {
    # /proc/mounts, not findmnt: findmnt lives on the filesystem we are watching
    # and was itself unreadable ("/usr/bin/findmnt: Input/output error") in the
    # 2026-08-23 capture. Anything this script needs must already be in RAM.
    #
    # emergency_ro is NOT optional to check. In the second capture, with the
    # journal aborted and every write failing, /proc/mounts still read:
    #
    #   /dev/mmcblk0p3 / ext4 rw,noatime,errors=remount-ro,emergency_ro 0 0
    #
    # Still "rw". Modern ext4 sets an emergency-read-only SUPERBLOCK flag rather
    # than rewriting the mount flags, so the naive rw/ro test sails straight
    # past the exact event this whole script exists to catch. That is why
    # errors_count is also a trigger below -- two independent detectors, because
    # the first one was wrong and nothing would have told us.
    /usr/bin/awk '$2=="/" {
        if ($4 ~ /(^|,)ro(,|$)/ || $4 ~ /emergency_ro/) print "ro"; else print "rw"; exit }' \
        /proc/mounts 2>/dev/null || printf 'unknown'
}

# A 512-byte O_DIRECT read from the eMMC -- the sibling controller on the SAME
# EDMA. THIS IS THE DISCRIMINATOR the first two captures could not provide,
# because mmc1 was idle both times and the log therefore could not say whether
# the shared engine had died or only mmc0.
#
#   mmc0 dies, this still reads ok  -> EDMA and the SDHCI IP are healthy; the
#                                      fault is specific to mmc0, its card, its
#                                      slot or its supply. The EDMA theory ends.
#   both die together               -> shared engine. The EDMA theory is the
#                                      answer and the DT overlay is worth it.
#
# iflag=direct matters: without it the page cache answers and the read proves
# nothing. Running it every tick also keeps /usr/bin/dd hot in the page cache,
# which is why it still works after the root fs has stopped serving reads --
# in the second capture /usr/bin/dmesg, findmnt, tail, nc and scp were all
# already unreadable.
emmc_probe() {
    [ -b /dev/mmcblk1 ] || { printf -- '-'; return; }
    if /usr/bin/dd if=/dev/mmcblk1 of=/dev/null bs=512 count=1 iflag=direct \
        >/dev/null 2>&1; then printf 'ok'; else printf 'FAIL'; fi
}

mmc0_state() {
    for d in /sys/class/mmc_host/mmc0/mmc0:*; do
        [ -d "$d" ] && { printf 'ok'; return; }
    done
    printf 'gone'
}

# Link state and address for the wifi dongle. Present because "sometimes it
# won't pick up an IP at boot" is unobservable by definition -- no IP means no
# ssh means no way to look. The beacon runs from t=0 and writes to kmsg, which
# survives in RAM, so a bad boot documents itself and the record is waiting
# whenever you next get in (over the USB gadget, see below).
#
# The boot ordering here is genuinely tight, and the logs show it: the dongle
# enumerates on USB at t≈3.4-4.3 s, but its driver does not bind until
# t≈56-57 s -- a 53-SECOND gap, consistent across both captured boots -- and
# association lands at t≈59-60 s. Anything that wants the network before that
# finds no interface at all. netconsole did exactly this and reported success
# anyway (§2). A DHCP client that gives up early would look identical from
# outside: no address, no entry on the router.
IFACE="${IFACE:-wlan0}"

net_state() {
    [ -r "/sys/class/net/$IFACE/operstate" ] || { printf 'absent'; return; }
    read -r s < "/sys/class/net/$IFACE/operstate" 2>/dev/null && printf '%s' "$s" \
        || printf '?'
}

# /usr/sbin is not on the debian user's PATH (systemd gives root a fuller one,
# but probe anyway rather than assume). Calling it every tick also keeps it in
# page cache, which is what makes it still work once the root fs stops serving
# reads -- the same reason the eMMC probe uses dd.
ipv4() {
    for b in /usr/sbin/ip /sbin/ip /usr/bin/ip; do
        [ -x "$b" ] || continue
        a="$("$b" -4 -o addr show dev "$IFACE" 2>/dev/null |
             /usr/bin/awk '{ split($4, p, "/"); print p[1]; exit }')"
        [ -n "$a" ] && { printf '%s' "$a"; return; }
        printf -- '-'; return
    done
    printf -- '?'
}

ext4_errors() {
    for f in /sys/fs/ext4/mmcblk0p3/errors_count; do
        [ -r "$f" ] && { read -r n < "$f" 2>/dev/null && printf '%s' "$n" && return; }
    done
    printf -- '-'
}

snapshot() {
    emit "ro_beacon SNAPSHOT begin"
    if [ -r "$SUMMARY" ]; then
        while IFS= read -r line; do emit "ro_beacon dma| $line"; done < "$SUMMARY"
    fi
    for f in /sys/class/mmc_host/mmc0/mmc0:*/type /sys/class/mmc_host/mmc0/mmc0:*/state; do
        [ -r "$f" ] && { read -r v < "$f" 2>/dev/null; emit "ro_beacon mmc0| ${f##*/}=$v"; }
    done
    emit "ro_beacon SNAPSHOT end"
}

noip_said=""
prev_root=""
prev_e4=""
p_mmc0=0; p_mmc1=0; p_i2c=0; p_ccerr=0; p_ccint=0
first=1

emit "ro_beacon started interval=${INTERVAL}s pid=$$"

while :; do
    read -r up _ < /proc/uptime
    read -r l1 _ < /proc/loadavg
    v="$(batt_v)"
    dma="$(dma_state)"
    root="$(root_mode)"
    mmc0="$(mmc0_state)"
    emmc="$(emmc_probe)"
    net="$(net_state)"
    ip4="$(ipv4)"
    e4="$(ext4_errors)"
    set -- $(irq_counts)
    c_mmc0="$1"; c_mmc1="$2"; c_i2c="$3"; c_ccerr="$4"; c_ccint="$5"

    if [ "$first" = 1 ]; then
        d_mmc0=0; d_mmc1=0; d_i2c=0; d_ccerr=0; d_ccint=0; first=0
    else
        d_mmc0=$(( c_mmc0 - p_mmc0 )); d_mmc1=$(( c_mmc1 - p_mmc1 ))
        d_i2c=$(( c_i2c - p_i2c ));    d_ccerr=$(( c_ccerr - p_ccerr ))
        d_ccint=$(( c_ccint - p_ccint ))
    fi
    p_mmc0="$c_mmc0"; p_mmc1="$c_mmc1"; p_i2c="$c_i2c"
    p_ccerr="$c_ccerr"; p_ccint="$c_ccint"

    emit "ro_beacon up=${up} v=${v} ld=${l1} root=${root} mmc0=${mmc0} emmc=${emmc} net=${net} ip=${ip4} e4err=${e4} dma=${dma} ccerr=${c_ccerr} dccerr=${d_ccerr} dccint=${d_ccint} dmmc0=${d_mmc0} dmmc1=${d_mmc1} di2c=${d_i2c}"

    # A boot that never gets an address is the thing you cannot see from
    # outside, so make it announce itself once, loudly, in the log that
    # survives. 180 s is well past the ~60 s the dongle normally needs.
    case "$up" in *.*) up_i="${up%%.*}" ;; *) up_i="$up" ;; esac
    if [ -z "$noip_said" ] && [ "$up_i" -gt 180 ] 2>/dev/null && [ "$ip4" = "-" ]; then
        emit "ro_beacon EVENT no IPv4 on ${IFACE} after ${up_i}s (operstate=${net}) -- associated but no lease, or never associated"
        noip_said=1
    fi

    # Two independent triggers. The rw->ro one would have MISSED the second
    # 2026-08-23 event outright (see root_mode) -- errors_count is the one that
    # actually fires, and both are cheap.
    fired=""
    [ -n "$prev_root" ] && [ "$root" != "$prev_root" ] && fired="root ${prev_root} -> ${root}"
    [ -z "$fired" ] && [ -n "$prev_e4" ] && [ "$e4" != "$prev_e4" ] && fired="ext4 errors_count ${prev_e4} -> ${e4}"
    # An eMMC read failing is a finding on its own even if the root fs is fine:
    # it means the shared EDMA is sick, not the SD card.
    [ -z "$fired" ] && [ "$emmc" = FAIL ] && fired="eMMC read FAILED"

    if [ -n "$fired" ]; then
        i=0
        while [ "$i" -lt 3 ]; do
            emit "ro_beacon EVENT ${fired} at up=${up} v=${v} emmc=${emmc} ccerr=${c_ccerr} dma=${dma}"
            i=$(( i + 1 ))
        done
        snapshot
    fi
    prev_root="$root"; prev_e4="$e4"

    sleep "$INTERVAL"
done
