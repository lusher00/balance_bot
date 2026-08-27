#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# clone_to_emmc.sh -- put a bootable copy of the running SD root onto the eMMC,
#                     so a card that will not initialise stops being a brick.
#
#   sudo ./tools/clone_to_emmc.sh          DRY RUN. Measures, plans, prints, exits.
#   sudo ./tools/clone_to_emmc.sh --go     actually does it. Destroys mmcblk1.
#
# ---- Why -------------------------------------------------------------------
#
# From the 2026-08-23 serial capture, on a power-on reset:
#
#   Loading Environment from EXT4... Card did not respond to voltage select!
#   ** No partition table - mmc 1 **
#
# The SD failed voltage negotiation in U-Boot, and there was nothing else to
# boot. U-Boot fell through to PXE over USB RNDIS and hung. The eMMC's
# partition table is gone -- consistent with the
# `dd if=/dev/zero of=/dev/mmcblk1 bs=1M count=10` in shell history, and with
# the kernel never printing `mmcblk1: p1 p2 p3`.
#
# The eMMC is soldered down. Whatever is wrong with the SD path -- socket
# contacts, solder, the 3.3 V feed -- the eMMC does not share the socket. That
# makes it the right recovery device, not just a spare.
#
# ---- Read this before running it -------------------------------------------
#
# 1. THE BOOT ORDER CHANGES. The AM335x ROM tries eMMC before the SD card.
#    Right now the eMMC has no valid bootloader, so the ROM skips it and boots
#    the SD. Once this script finishes, THE BOARD BOOTS FROM eMMC by default,
#    SD card present or not. That is the point -- but it means the SD root you
#    have been running stops being the live one. Know which you are on:
#        findmnt -no SOURCE /        (or: cat /proc/mounts | grep ' / ')
#    To boot the SD again, hold the S2 / BOOT button through power-on.
#
# 2. IT MAY NOT FIT. The eMMC is 3.56 GiB. The SD root is on a 14.8 GiB card.
#    This script measures before it touches anything and refuses rather than
#    filling the target and leaving you with a half-copy that boots to a
#    prompt. If it refuses, thin the root out and re-run.
#
# 3. NO SWAP PARTITION IS CREATED, deliberately. Swap currently sits on
#    /dev/mmcblk0p2 -- the same device that dies -- which is why a storage
#    failure turned into `journalctl` segfaulting and half of /usr/bin
#    returning EIO: any page faulted back from a dead swap device is a SIGBUS.
#    zram0 is already active at priority 100 (246 MB), and that is enough on a
#    512 MB board. Leaving swap off the eMMC is a fix, not an omission.
#
# 4. It stops the bot services first. A root filesystem being rsynced while
#    balance_bot writes robot.conf is a copy of a moment that never existed.
#
# 5. /tmp IS EXCLUDED, and that is not a detail. /tmp is NOT a tmpfs on this
#    image -- there is no /tmp line in /proc/mounts, so it lives on the SD
#    card -- and the 2026-08-24 partial archive found this in it:
#
#        /tmp/balance_bot.log    577,657,108 bytes    3,452,153 lines
#
#    Overwhelmingly one message, repeating every 16-20 ms:
#
#        [WARN][659610] motor_hal_roboclaw: encoder read failed (-2)
#        [WARN][659630] motor_hal_roboclaw: encoder read failed (-2)
#
#    551 MB of unthrottled warnings, written to the card that keeps failing.
#    Copying it to a 3.56 GiB eMMC would be absurd, and it is very likely the
#    difference between fitting and not. It is also a direct correction to the
#    claim in HANDOFF §1.2 that "an idle instance writes essentially nothing" --
#    that was inference, and it was wrong.
set -u

SRC_DISK=/dev/mmcblk0
DST_DISK=/dev/mmcblk1
BOOT_MB=96
MNT=/mnt/emmc_clone

GO=0
case "${1:-}" in
    --go) GO=1 ;;
    "")   GO=0 ;;
    *) echo "  unknown option: $1 (only --go)" >&2; exit 1 ;;
esac

die()  { echo "  error: $*" >&2; exit 1; }
note() { echo "  $*"; }
step() { echo; echo "==> $*"; }

[ "$(id -u)" = 0 ] || die "must run as root (sudo)"

# ---- refuse to do anything stupid ------------------------------------------

root_src="$(/usr/bin/awk '$2=="/" {print $1; exit}' /proc/mounts)"
case "$root_src" in
    ${SRC_DISK}p*) : ;;
    *) die "root is on '$root_src', not on $SRC_DISK.
       This script clones the SD root ONTO the eMMC. If you are already
       running from the eMMC it would be copying the target onto itself." ;;
esac
[ -b "$DST_DISK" ] || die "$DST_DISK does not exist"
case "$root_src" in
    ${DST_DISK}*) die "root is on $DST_DISK -- refusing to overwrite the running system" ;;
esac

for t in sfdisk mkfs.ext4 mkfs.vfat rsync blkid; do
    command -v "$t" >/dev/null 2>&1 || \
        die "missing '$t'. Install it first (mkfs.vfat is in dosfstools)."
done

# ---- measure ---------------------------------------------------------------

step "Measuring"

dst_bytes=$(( $(cat "/sys/class/block/$(basename "$DST_DISK")/size") * 512 ))
boot_bytes=$(( BOOT_MB * 1024 * 1024 ))
# 4 MiB reserved at the front for the raw bootloader region, plus slack for
# ext4 metadata and the reserved-blocks percentage.
avail_bytes=$(( dst_bytes - boot_bytes - 4*1024*1024 ))
avail_bytes=$(( avail_bytes * 94 / 100 ))

# -x stays on one filesystem, so /proc /sys /dev /run and the eMMC mount are
# all excluded automatically. The journal is excluded because it is the one
# directory that is both large and worthless in a recovery image.
note "measuring used space on / (a few seconds) ..."
used_kb="$(/usr/bin/du -shxk --exclude=/var/log/journal --exclude=/tmp / 2>/dev/null | /usr/bin/awk '{print $1}')"
used_bytes=$(( used_kb * 1024 ))
tmp_kb="$(/usr/bin/du -sxk /tmp 2>/dev/null | /usr/bin/awk '{print $1}')"
[ -n "${tmp_kb:-}" ] && [ "$tmp_kb" -gt 51200 ] && \
    note "note: /tmp holds $(/usr/bin/awk -v b=$((tmp_kb*1024)) 'BEGIN{printf "%.2f GiB", b/1073741824}') and is NOT being copied (see header item 5)"

hum() { /usr/bin/awk -v b="$1" 'BEGIN{ printf "%.2f GiB", b/1073741824 }'; }

note "source root in use : $(hum "$used_bytes")"
note "eMMC capacity      : $(hum "$dst_bytes")"
note "usable for root    : $(hum "$avail_bytes")  (after ${BOOT_MB}MiB boot + slack)"

if [ "$used_bytes" -gt "$avail_bytes" ]; then
    echo
    die "IT DOES NOT FIT -- short by $(hum $(( used_bytes - avail_bytes ))).
       Nothing has been touched. Thin the root out and re-run. Usual suspects:
         sudo journalctl --vacuum-size=16M
         sudo apt clean
         du -shx /home/debian/* /usr/* /var/* 2>/dev/null | sort -h | tail -20"
fi
note "fits, with $(hum $(( avail_bytes - used_bytes ))) to spare"

# The raw region ahead of the first partition is where MLO and u-boot.img live
# on these images. Copy it verbatim rather than trying to locate the files.
p1_start="$(sfdisk -d "$SRC_DISK" | /usr/bin/awk '/p1 /{ for(i=1;i<=NF;i++) if ($i=="start=") print $(i+1); }' | tr -d ',')"
[ -z "$p1_start" ] && p1_start="$(sfdisk -d "$SRC_DISK" | /usr/bin/awk -F'start=' '/p1 /{split($2,a,","); gsub(/ /,"",a[1]); print a[1]; exit}')"
[ -n "$p1_start" ] || die "could not read the start sector of ${SRC_DISK}p1"
[ "$p1_start" -ge 2048 ] || die "${SRC_DISK}p1 starts at sector $p1_start -- too early for a raw bootloader region"
note "raw bootloader region: sectors 1..$(( p1_start - 1 ))  ($(hum $(( (p1_start-1)*512 ))))"

# ---- plan ------------------------------------------------------------------

step "Plan"
cat <<PLAN
  1. stop balance_bot, balance_bot_server, bbb_oled, batt_monitor
  2. wipe and repartition $DST_DISK
       p1  ${BOOT_MB}MiB  vfat, bootable   (starts at sector $p1_start)
       p2  rest           ext4, label rootfs
  3. copy sectors 1..$(( p1_start - 1 )) from $SRC_DISK  (MLO / u-boot, raw)
  4. mkfs both, mount at $MNT
  5. rsync / -> $MNT   (one filesystem, no journal)
  6. copy ${SRC_DISK}p1 contents -> ${DST_DISK}p1
  7. rewrite $MNT/etc/fstab onto the new UUIDs, swap line dropped
PLAN

if [ "$GO" != 1 ]; then
    echo
    note "DRY RUN -- nothing was changed. Re-run with --go to do it."
    exit 0
fi

echo
printf "  This ERASES %s. Type ERASE to continue: " "$DST_DISK"
read -r ans
[ "$ans" = "ERASE" ] || die "aborted"

# ---- do it -----------------------------------------------------------------

step "Stopping services"
/usr/bin/systemctl stop balance_bot_server balance_bot bbb_oled batt_monitor 2>/dev/null
sync

step "Partitioning $DST_DISK"
/usr/bin/umount "${DST_DISK}"p* 2>/dev/null
wipefs -a "$DST_DISK" >/dev/null 2>&1
sfdisk "$DST_DISK" <<SFD || die "sfdisk failed"
label: dos
${DST_DISK}p1 : start=$p1_start, size=$(( BOOT_MB * 2048 )), type=c, bootable
${DST_DISK}p2 : type=83
SFD
partprobe "$DST_DISK" 2>/dev/null || true
sleep 2
[ -b "${DST_DISK}p2" ] || die "${DST_DISK}p2 did not appear"

step "Copying the raw bootloader region"
dd if="$SRC_DISK" of="$DST_DISK" bs=512 skip=1 seek=1 count=$(( p1_start - 1 )) \
   conv=notrunc status=none || die "raw bootloader copy failed"

step "Making filesystems"
mkfs.vfat -F 16 -n BOOT "${DST_DISK}p1" >/dev/null || die "mkfs.vfat failed"
mkfs.ext4 -F -L rootfs "${DST_DISK}p2" >/dev/null || die "mkfs.ext4 failed"

step "Copying the root filesystem (this is the slow part)"
mkdir -p "$MNT"
mount "${DST_DISK}p2" "$MNT" || die "cannot mount ${DST_DISK}p2"
rsync -aHAXx --info=progress2 \
      --exclude='/var/log/journal/*' \
      --exclude='/var/cache/apt/archives/*' \
      --exclude='/tmp/*' \
      --exclude="$MNT" \
      / "$MNT/" || die "rsync failed"
mkdir -p "$MNT/proc" "$MNT/sys" "$MNT/dev" "$MNT/run" "$MNT/tmp" "$MNT/mnt"
chmod 1777 "$MNT/tmp"

step "Copying the boot partition"
mkdir -p "$MNT/boot/firmware"
mount "${DST_DISK}p1" "$MNT/boot/firmware" || die "cannot mount ${DST_DISK}p1"
if [ -d /boot/firmware ] && [ -n "$(ls -A /boot/firmware 2>/dev/null)" ]; then
    cp -a /boot/firmware/. "$MNT/boot/firmware/" || die "boot copy failed"
else
    note "WARNING: /boot/firmware is empty on the source. If the raw u-boot"
    note "region does not carry the bootloader on this image, the eMMC will"
    note "not boot. Check before trusting it."
fi

step "Rewriting fstab"
root_uuid="$(blkid -s UUID -o value "${DST_DISK}p2")"
boot_uuid="$(blkid -s UUID -o value "${DST_DISK}p1")"
[ -n "$root_uuid" ] || die "could not read the new root UUID"
{
    echo "# Written by tools/clone_to_emmc.sh -- eMMC root."
    echo "# No swap line: swap on the SD is what turned storage failures into"
    echo "# SIGBUS. zram0 covers it. See the script header."
    echo "UUID=$root_uuid  /               ext4  noatime,errors=remount-ro  0  1"
    echo "UUID=$boot_uuid  /boot/firmware  vfat  defaults                   0  2"
    # Keep any non-device lines the original had (the debugfs gid=gpio one).
    /usr/bin/awk '$1 !~ /^(#|UUID=|\/dev\/)/ && NF >= 3 { print }' /etc/fstab
} > "$MNT/etc/fstab"
note "new root UUID: $root_uuid"

sync
umount "$MNT/boot/firmware" "$MNT" 2>/dev/null

step "Done"
cat <<DONE
  The eMMC now has a bootable copy.

  IMPORTANT -- the ROM prefers eMMC. On the next power cycle the board boots
  from the eMMC whether or not the SD is in the slot. Confirm which root you
  landed on:

      findmnt -no SOURCE /            expect ${DST_DISK}p2

  To boot the SD deliberately, hold the S2 / BOOT button through power-on.

  The SD card is now a spare, and the socket problem in §0.5 stops being able
  to take the board down. Keep the card -- it is still the only copy of
  anything that was not in this rsync.
DONE
