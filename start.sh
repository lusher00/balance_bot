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

SERVER_JS="/home/debian/cat-follower-server/server.js"
NODE_BIN="/usr/local/bin/node"
BALANCE_BOT="/usr/local/bin/balance_bot"

# Default args — override with positional args
BOT_ARGS="-i sbus ${@}"

# ── sanity checks ──────────────────────────────────────────────────
if [ ! -f "$SERVER_JS" ]; then
    echo "ERROR: server.js not found at $SERVER_JS"
    exit 1
fi

if [ ! -x "$BALANCE_BOT" ]; then
    echo "ERROR: balance_bot not found at $BALANCE_BOT — run: sudo make install"
    exit 1
fi

# ── start server.js in background ─────────────────────────────────
echo "Starting server.js..."
"$NODE_BIN" "$SERVER_JS" &
SERVER_PID=$!

# Give it a moment to bind its port
sleep 0.5

if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "ERROR: server.js failed to start"
    exit 1
fi

echo "server.js running (pid $SERVER_PID)"

# ── cleanup on exit ────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Stopping server.js (pid $SERVER_PID)..."
    kill "$SERVER_PID" 2>/dev/null
    wait "$SERVER_PID" 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

# ── run balance_bot in foreground (ncurses display takes over) ─────
echo "Starting balance_bot $BOT_ARGS"
sleep 0.5   # let socket settle before bot tries to connect
sudo TERM="$TERM" "$BALANCE_BOT" $BOT_ARGS
