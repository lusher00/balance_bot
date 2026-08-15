#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# Install bbb_oled.py and its systemd unit. Run on the BeagleBone.
#
#   ./install.sh                 install, enable and start
#   ./install.sh --no-start      install only
#   ./install.sh --skip-deps     install even if the Python imports are missing
#   ./install.sh --uninstall     remove everything except /etc/default/bbb-oled
#
# Safe to re-run. /etc/default/bbb-oled is never overwritten once it exists.

set -euo pipefail

SCRIPT_SRC="bbb_oled.py"
UNIT_SRC="bbb-oled.service"
DEFAULT_SRC="bbb-oled.default.example"

SCRIPT_DST="/usr/local/bin/bbb_oled.py"
UNIT_DST="/etc/systemd/system/bbb-oled.service"
DEFAULT_DST="/etc/default/bbb-oled"
SERVICE="bbb-oled"

cd "$(dirname "$0")"

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33m==> %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[31m==> %s\033[0m\n' "$*" >&2; exit 1; }

need_root() {
    [ "$(id -u)" -eq 0 ] || die "run with sudo: sudo $0 $*"
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
        -h|--help)   sed -n '4,13p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)           die "unknown option: $arg" ;;
    esac
done

need_root "$@"

for f in "$SCRIPT_SRC" "$UNIT_SRC" "$DEFAULT_SRC"; do
    [ -f "$f" ] || die "missing $f — run this from the oled-utils directory"
done

# ── preflight ───────────────────────────────────────────────────────
# Check things that otherwise fail later as confusing runtime errors.

say "Checking prerequisites"

RUN_USER="$(awk -F= '/^User=/{print $2}' "$UNIT_SRC")"
RUN_USER="${RUN_USER:-debian}"
if ! id "$RUN_USER" >/dev/null 2>&1; then
    die "unit runs as '$RUN_USER' but that user does not exist"
fi

# Import-check as the user the SERVICE runs as, not as root.
#
# This script needs sudo to write to /usr/local/bin and /etc, so a bare
# `python3 -c` here runs as root. Packages installed with `pip3 install --user`
# live in ~/.local and are invisible to root, so checking as root reports them
# missing for a setup that would have started fine. Check the account that will
# actually do the importing.
DEPCHECK='
import sys
missing = []
for mod, pkg in (("luma.oled", "luma.oled"), ("PIL", "pillow")):
    try:
        __import__(mod)
    except ImportError:
        missing.append(pkg)
print("  interpreter: " + sys.executable)
if missing:
    print("  MISSING: " + ", ".join(missing))
    sys.exit(1)
print("  luma.oled and pillow present")
'
echo "  checking as user: $RUN_USER"
if ! runuser -u "$RUN_USER" -- python3 -c "$DEPCHECK" 2>/dev/null \
     && ! sudo -u "$RUN_USER" python3 -c "$DEPCHECK"; then
    echo
    warn "Dependencies missing for '$RUN_USER' — the user the service runs as."
    echo "  Install for that user only:"
    echo "      sudo -u $RUN_USER pip3 install --user --break-system-packages luma.oled pillow"
    echo "  Or system-wide (also covers root and any other user):"
    echo "      sudo pip3 install --break-system-packages luma.oled pillow"
    echo
    echo "  On Debian you may prefer the packaged build, which avoids pip entirely:"
    echo "      sudo apt install python3-pil"
    echo "      sudo pip3 install --break-system-packages luma.oled"
    echo
    if [ "$SKIP_DEPS" -eq 1 ]; then
        warn "--skip-deps given; installing anyway. The service will fail to start"
        warn "until the imports resolve."
    else
        die "missing Python dependencies (re-run with --skip-deps to install regardless)"
    fi
fi

if getent group i2c >/dev/null; then
    if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx i2c; then
        echo "  $RUN_USER is in the i2c group"
    else
        say "Adding $RUN_USER to the i2c group"
        usermod -aG i2c "$RUN_USER"
        warn "$RUN_USER was just added to 'i2c'. The service picks this up via"
        warn "SupplementaryGroups= immediately, but that user's existing login"
        warn "shells will not until they log out and back in."
    fi
else
    warn "no 'i2c' group on this system; relying on the device node's mode"
fi

# The unit has ConditionPathExists=/dev/i2c-1, so a missing bus shows up as
# "condition failed" rather than a crash loop. Still worth saying now.
PORT="1"
[ -f "$DEFAULT_DST" ] && PORT="$(sed -n 's/.*--i2c-port \([0-9]\+\).*/\1/p' "$DEFAULT_DST" | head -1)"
PORT="${PORT:-1}"
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
