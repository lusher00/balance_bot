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
# Install the OLED status display and its systemd unit. Run on the BeagleBone.
#
#   ./install.sh                 install, enable and start
#   ./install.sh --no-start      install only
#   ./install.sh --skip-deps     install even if the Python imports are missing
#   ./install.sh --uninstall     remove everything except /etc/default/bbb_oled
#
# Safe to re-run. /etc/default/bbb_oled is never overwritten once it exists.


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
        "sudo " "${BASH_SOURCE[0]}" >&2
    return 1
fi

set -euo pipefail

# The script is vendored from ~/oled-utils (upstream); keep them identical:
#     cmp oled_status.py ~/oled-utils/oled_status.py
# It installs under the OLD name and unit on purpose -- /etc/default/bbb_oled,
# the dashboard's /run/bbb_oled/status.json path and every note in HANDOFF.md
# are written against "bbb_oled", and renaming a working service to match an
# upstream filename buys nothing.
SCRIPT_SRC="oled_status.py"
UNIT_SRC="bbb_oled.service"
DEFAULT_SRC="bbb_oled.default.example"

SCRIPT_DST="/usr/local/bin/bbb_oled.py"
UNIT_DST="/etc/systemd/system/bbb_oled.service"
DEFAULT_DST="/etc/default/bbb_oled"
SERVICE="bbb_oled"

cd "$(dirname "${BASH_SOURCE[0]}")"

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33m==> %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[31m==> %s\033[0m\n' "$*" >&2; exit 1; }

need_root() {
    [ "$(id -u)" -eq 0 ] || die "run with sudo: sudo ${BASH_SOURCE[0]} $*"
}

do_uninstall() {
    need_root "$@"
    say "Stopping and disabling $SERVICE"
    systemctl disable --now "$SERVICE" 2>/dev/null || true
    rm -f "$UNIT_DST" "$SCRIPT_DST"
    systemctl daemon-reload
    say "Removed $UNIT_DST and $SCRIPT_DST"
    say "Left $DEFAULT_DST in place — delete it by hand if you want it gone."
    exit 0
}

START=1
SKIP_DEPS=0
for arg in "$@"; do
    case "$arg" in
        --uninstall) do_uninstall "$@" ;;
        --no-start)  START=0 ;;
        --skip-deps) SKIP_DEPS=1 ;;
        -h|--help)   sed -n '4,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)           die "unknown option: $arg" ;;
    esac
done

need_root "$@"

for f in "$SCRIPT_SRC" "$UNIT_SRC" "$DEFAULT_SRC"; do
    [ -f "$f" ] || die "missing $f — run this from the oled-utils directory"
done

# ── migrate from the old hyphenated name ─────────────────────────────────
# The service used to be bbb-oled; every other bot service uses an underscore
# (balance_bot, balance_bot_server, batt_monitor), so it was renamed. Without
# this the old unit stays installed AND enabled, and you end up with two
# daemons driving one I2C display -- which looks exactly like "the service is
# running but the screen is dead", because they fight over the bus.
OLD_UNIT="/etc/systemd/system/bbb-oled.service"
OLD_DEFAULT="/etc/default/bbb-oled"
if [ -f "$OLD_UNIT" ]; then
    warn "found the old bbb-oled.service — stopping, disabling and removing it"
    systemctl disable --now bbb-oled 2>/dev/null || true
    rm -f "$OLD_UNIT"
    systemctl daemon-reload
fi
if [ -f "$OLD_DEFAULT" ] && [ ! -f "$DEFAULT_DST" ]; then
    say "migrating $OLD_DEFAULT -> $DEFAULT_DST (keeping your options)"
    mv "$OLD_DEFAULT" "$DEFAULT_DST"
elif [ -f "$OLD_DEFAULT" ]; then
    warn "$OLD_DEFAULT still exists but $DEFAULT_DST is already there;"
    warn "leaving both — delete the old one by hand once you have compared them"
fi

# NOTE ON ORDERING: this runs BEFORE the dependency preflight below, which can
# die(). Cleaning up the old unit must not be conditional on the new one
# installing successfully -- if it were, a missing Python module would leave the
# board with the old bbb-oled still enabled AND the new bbb_oled absent, which
# is worse than either alone.


# ── preflight ───────────────────────────────────────────────────────
# Check things that otherwise fail later as confusing runtime errors.

say "Checking prerequisites"

RUN_USER="$(awk -F= '/^User=/{print $2}' "$UNIT_SRC")"
RUN_USER="${RUN_USER:-debian}"
if ! id "$RUN_USER" >/dev/null 2>&1; then
    die "unit runs as '$RUN_USER' but that user does not exist"
fi

# Import-check the way the SERVICE will resolve it, not the way your shell does.
#
# This has now been wrong in both directions, so it is worth being precise.
#
# v1 asked "can debian import luma?" via runuser. That passed while the service
# died on ImportError every 5s, because `pip3 install --user` puts the modules
# in ~/.local and the unit sets ProtectHome=yes, which hands the service an
# empty /home.
#
# v2 then rejected any module resolving under /home. That FAILED INSTALLS THAT
# WOULD HAVE WORKED: if a package is installed both system-wide and in
# ~/.local, the user site shadows the system copy, so find_spec run as debian
# reports the /home path -- while systemd, with /home emptied, happily falls
# back to the system copy. Observed here: this check refused to install while
# the running service was importing luma without complaint.
#
# The question is not "where does the login user find it" but "can it still be
# found with ~/.local removed from sys.path". PYTHONNOUSERSITE=1 does exactly
# that, and is a faithful stand-in for what ProtectHome=yes leaves visible.
DEPCHECK='
import sys, importlib.util as u
missing = []
for mod, pkg in (("luma.oled", "luma.oled"), ("PIL", "pillow")):
    try:
        s = u.find_spec(mod)
    except Exception:
        s = None
    if s is None:
        missing.append(pkg)
        print("    %-10s MISSING" % mod)
        continue
    p = s.origin
    if not p and s.submodule_search_locations:
        p = list(s.submodule_search_locations)[0]
    print("    %-10s %s" % (mod, p or "(namespace package)"))
if missing:
    print("    MISSING: " + ", ".join(missing))
    sys.exit(1)
'
echo "  your shell's view (as $RUN_USER, ~/.local included):"
runuser -u "$RUN_USER" -- python3 -c "$DEPCHECK" || true
echo "  the service's view (~/.local removed, as ProtectHome=yes leaves it):"
SVC_RC=0
runuser -u "$RUN_USER" -- env PYTHONNOUSERSITE=1 python3 -c "$DEPCHECK" || SVC_RC=$?
if [ "$SVC_RC" -ne 0 ]; then
    echo
    warn "The modules are not resolvable with ~/.local excluded, so systemd will"
    warn "not find them either -- the unit sets ProtectHome=yes."
    echo "  Install them system-wide:"
    echo "      sudo pip3 install --break-system-packages luma.oled pillow"
    echo
    echo "  On Debian you may prefer the packaged build for PIL:"
    echo "      sudo apt install python3-pil"
    echo "      sudo pip3 install --break-system-packages luma.oled"
    echo
    if [ "$SKIP_DEPS" -eq 1 ]; then
        warn "--skip-deps given; installing anyway. The service will fail to start"
        warn "until the imports resolve."
    else
        die "dependencies not visible to the service (re-run with --skip-deps to install regardless)"
    fi
fi

# Which bus? Read it from the options file BEFORE the group check, because the
# group check needs to stat the right device node. This used to sit further
# down, so the group lookup silently assumed bus 1.
PORT="1"
[ -f "$DEFAULT_DST" ] && PORT="$(sed -n 's/.*--i2c-port \([0-9]\+\).*/\1/p' "$DEFAULT_DST" | head -1)"
PORT="${PORT:-1}"

# Add the user to the group that ACTUALLY owns the bus node, not a hardcoded
# "i2c". This image ships /dev/i2c-* as root:gpio, and adding the user to a
# group that does not own the device achieves nothing while looking like it did.
BUS_DEV="/dev/i2c-$PORT"
DEV_GROUP="$(stat -c %G "$BUS_DEV" 2>/dev/null || echo i2c)"
if getent group "$DEV_GROUP" >/dev/null; then
    if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx "$DEV_GROUP"; then
        echo "  $RUN_USER is in the '$DEV_GROUP' group (owner of $BUS_DEV)"
    else
        say "Adding $RUN_USER to the '$DEV_GROUP' group (owner of $BUS_DEV)"
        usermod -aG "$DEV_GROUP" "$RUN_USER"
        warn "$RUN_USER was just added to '$DEV_GROUP'. The service picks this up via"
        warn "SupplementaryGroups= immediately, but that user's existing login"
        warn "shells will not until they log out and back in."
    fi
else
    warn "no '$DEV_GROUP' group on this system; relying on the device node's mode"
fi

# The unit has ConditionPathExists=/dev/i2c-1, so a missing bus shows up as
# "condition failed" rather than a crash loop. Still worth saying now.
if [ ! -e "/dev/i2c-$PORT" ]; then
    warn "/dev/i2c-$PORT does not exist — the service will not start until it does."
    warn "On the Blue that usually means the I2C1 overlay is not enabled in /boot/uEnv.txt."
fi

# ── install ─────────────────────────────────────────────────────────

say "Installing $SCRIPT_DST"
install -m 0755 -o root -g root "$SCRIPT_SRC" "$SCRIPT_DST"

say "Installing $UNIT_DST"
install -m 0644 -o root -g root "$UNIT_SRC" "$UNIT_DST"

if [ -f "$DEFAULT_DST" ]; then
    say "Keeping existing $DEFAULT_DST"
    echo "  current: $(grep -h '^OLED_ARGS' "$DEFAULT_DST" || echo '(no OLED_ARGS set)')"
else
    say "Installing $DEFAULT_DST"
    install -m 0644 -o root -g root "$DEFAULT_SRC" "$DEFAULT_DST"
fi

say "Validating the unit"
systemd-analyze verify "$UNIT_DST" || die "unit failed validation — not enabling it"

systemctl daemon-reload

if [ "$START" -eq 1 ]; then
    say "Enabling and starting $SERVICE"
    systemctl enable "$SERVICE" >/dev/null
    systemctl restart "$SERVICE"
    sleep 2
    if systemctl is-active --quiet "$SERVICE"; then
        say "$SERVICE is running"
    else
        warn "$SERVICE did not come up. Recent log:"
        journalctl -u "$SERVICE" -n 20 --no-pager || true
        exit 1
    fi
else
    say "Installed but not started (--no-start). Start with:"
    echo "    sudo systemctl enable --now $SERVICE"
fi

cat <<EOF

Done.

  Edit options:  sudo nano $DEFAULT_DST  &&  sudo systemctl restart $SERVICE
  Watch logs:    journalctl -u $SERVICE -f
  Test by hand:  $SCRIPT_DST --once -v
  Find the panel: i2cdetect -y -r $PORT     (expect 3c, or 3d on some modules)
EOF
