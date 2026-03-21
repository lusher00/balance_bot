#!/bin/bash
# start.sh — launch balance_bot + server.js for a bench session
#
# Usage:
#   ./start.sh                  # default: sbus input, system status display
#   ./start.sh -d sbus          # add SBUS TX display
#   ./start.sh -d sbus -d pid   # SBUS + PID display
#   ./start.sh -d all           # all display blocks
#   ./start.sh -i xbox          # xbox input instead of sbus
#
# All extra args are passed straight through to balance_bot.
# server.js runs in the background and is killed on exit.

SERVER_JS="/home/debian/balance_bot/server/server.js"
NODE_BIN="/usr/local/bin/node"
BALANCE_BOT="/usr/local/bin/balance_bot"
SERVER_LOG="/tmp/server.log"

# Default args — no input source (arm via iPhone or MODE button)
# Override on command line e.g: ./start.sh -i sbus -u /dev/ttyO1
BOT_ARGS="-d all -m /dev/ttyO5 -B 460800 ${@}"

# ── sanity checks ──────────────────────────────────────────────────
if [ ! -f "$SERVER_JS" ]; then
    echo "ERROR: server.js not found at $SERVER_JS"
    exit 1
fi

if [ ! -x "$BALANCE_BOT" ]; then
    echo "ERROR: balance_bot not found at $BALANCE_BOT — run: sudo make install"
    exit 1
fi

# ── kill any stale balance_bot / server.js ────────────────────────
sudo systemctl stop balance_bot 2>/dev/null
sudo pkill -f balance_bot 2>/dev/null
pkill -f server.js 2>/dev/null
sleep 0.5

# ── start server.js in background, output to log only ─────────────
echo "Starting server.js (log: $SERVER_LOG)..."
"$NODE_BIN" "$SERVER_JS" > "$SERVER_LOG" 2>&1 &
SERVER_PID=$!

# Give it a moment to bind its port
sleep 0.5

if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "ERROR: server.js failed to start — check $SERVER_LOG"
    exit 1
fi

echo "server.js running (pid $SERVER_PID)"

# ── cleanup on exit ────────────────────────────────────────────────
cleanup() {
    kill "$SERVER_PID" 2>/dev/null
    wait "$SERVER_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

# ── run balance_bot in foreground (ncurses display takes over) ─────
echo "Starting balance_bot $BOT_ARGS"
sleep 0.5   # let socket settle before bot tries to connect
sudo TERM="$TERM" "$BALANCE_BOT" $BOT_ARGS
