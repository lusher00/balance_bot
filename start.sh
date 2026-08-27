#!/bin/bash
# start.sh — launch balance_bot for a bench session
#
# Usage:
#   ./start.sh                       # -i sbus, no extra display blocks
#   ./start.sh --input none          # CONTROL run: no input source at all
#   ./start.sh --input sbus          # TEST run: SBUS frames on ttyS5
#   ./start.sh --no-server           # skip server.js and the web dashboard
#   ./start.sh --display all         # -d all  (was unconditional; now opt-in)
#   ./start.sh --label "control 1"   # tag this run in the kernel log
#
# Anything else is passed straight through to balance_bot.
#
# ---- Why this was rewritten (2026-08-24) -----------------------------------
#
# It is now the instrument for the one open question, so it has to be precise
# about what it started. From §0.6 of HANDOFF.md: the fault kills the card in
# minutes when balance_bot runs with a live SBUS link, and the only two events
# with measured durations both had an IDLE SBUS UART and both lasted hours
# (33 min with `-i none`, 184 min with the transmitter off). Motors are ruled
# out — Ryan confirmed it happens with them unpowered.
#
# So the experiment is `--input none` versus `--input sbus`, and everything
# this script does either serves that comparison or gets out of its way.
#
# What changed and why each one mattered:
#
#   * INPUT IS NOW EXPLICIT AND ALWAYS PASSED. The old script only added
#     `-i sbus -u /dev/ttyS5` when you remembered `--sbus`, while its own usage
#     text claimed sbus was the default. So a plain `./start.sh` passed no -i
#     at all and left the input source up to whatever the binary defaults to.
#     For an experiment whose entire variable is the input source, that is not
#     survivable. It now always passes -i, and prints what it passed.
#
#   * IT STAMPS /dev/kmsg AT START AND END. A run whose configuration is only
#     in your terminal is a run you cannot identify later, and this fault eats
#     the filesystem the logs live on. The kernel ring buffer is in RAM, and
#     with tools/netconsole_setup.sh armed it leaves the board entirely. Every
#     capture from now on says which condition produced it, and the END line
#     gives the survival time without anyone holding a stopwatch.
#
#   * IT STOPS THE BRIDGE TOO. `systemctl stop balance_bot` alone does nothing
#     lasting: balance_bot_server.service has Requires=balance_bot.service, so
#     the bot gets pulled straight back in and you end up bench-testing against
#     a second instance you did not start.
#
#   * RESET BEFORE E-STOP CLEAR. These were the wrong way round. The sequence
#     that works is roboclaw_reset.py first, then estop_clear.sh — clearing a
#     latched e-stop before the RoboClaw has been reset does not take.
#
#   * `-d all` IS NO LONGER FORCED. It turns on every display block including
#     the D2/D3 traces, which were 54% of the 551 MB log. Fine when you want
#     them; not something a control run should carry silently.

set -u

BALANCE_BOT="/usr/local/bin/balance_bot"
SERVER_JS="/home/debian/balance_bot/server/server.js"
WEB_SCRIPT="/home/debian/balance_bot/web/serve_web.py"
SERVER_LOG="/tmp/server.log"
WEB_PORT=8888
RESET_PY="/home/debian/balance_bot/roboclaw_reset.py"
ESTOP_SH="/home/debian/balance_bot/estop_clear.sh"

INPUT=sbus
DISPLAY_ARG=""
WITH_SERVER=1
LABEL=""
PASSTHRU=()

while [ $# -gt 0 ]; do
    case "$1" in
        --input)    INPUT="${2:-}"; shift 2 ;;
        --sbus)     INPUT=sbus; shift ;;          # kept: old spelling
        --none)     INPUT=none; shift ;;
        --display)  DISPLAY_ARG="-d ${2:-all}"; shift 2 ;;
        --no-server) WITH_SERVER=0; shift ;;
        --label)    LABEL="${2:-}"; shift 2 ;;
        -h|--help)  sed -n '2,12p' "$0"; exit 0 ;;
        *)          PASSTHRU+=("$1"); shift ;;
    esac
done

case "$INPUT" in
    none) INPUT_ARGS="-i none" ;;
    sbus) INPUT_ARGS="-i sbus -u /dev/ttyS5" ;;
    xbox) INPUT_ARGS="-i xbox" ;;
    *)    echo "ERROR: --input must be none, sbus or xbox (got '$INPUT')" >&2; exit 1 ;;
esac

BOT_ARGS="-m /dev/ttyS1 -B 460800 $INPUT_ARGS $DISPLAY_ARG ${PASSTHRU[*]:-}"

# ── sanity checks ──────────────────────────────────────────────────
[ -x "$BALANCE_BOT" ] || {
    echo "ERROR: $BALANCE_BOT not found — run: sudo make install" >&2; exit 1; }

NODE_BIN=""
for n in /usr/bin/node /usr/local/bin/node; do [ -x "$n" ] && { NODE_BIN="$n"; break; }; done
if [ "$WITH_SERVER" = 1 ]; then
    [ -f "$SERVER_JS" ] || { echo "ERROR: server.js not found at $SERVER_JS" >&2; exit 1; }
    [ -n "$NODE_BIN" ]  || { echo "ERROR: no node binary found" >&2; exit 1; }
fi

# ── identify this run, in a log that survives the filesystem dying ──
RUN_ID="$(date +%s)"
STARTED="$(date -Is)"
stamp() { echo "<4>$*" | sudo tee /dev/kmsg >/dev/null 2>&1; }
stamp "bbot_run START id=$RUN_ID input=$INPUT server=$WITH_SERVER display='${DISPLAY_ARG:-none}' label='$LABEL'"

echo "  run id   : $RUN_ID   ($STARTED)"
echo "  input    : $INPUT    ${LABEL:+[$LABEL]}"
echo "  bot args : $BOT_ARGS"
[ "$INPUT" = none ] && echo "  (control run — ttyS5 is never opened, no SBUS frames)"

# ── clear anything already running ─────────────────────────────────
# Both units, and the bridge FIRST: balance_bot_server.service has
# Requires=balance_bot.service, so stopping the bot alone lets the bridge drag
# it straight back up underneath this session.
sudo systemctl stop balance_bot_server balance_bot 2>/dev/null
sudo pkill -f "$BALANCE_BOT" 2>/dev/null
pkill -f server.js    2>/dev/null
pkill -f serve_web.py 2>/dev/null
sleep 0.5

# ── RoboClaw: reset, THEN clear the latched e-stop ─────────────────
python3 "$RESET_PY"  || { echo "RoboClaw reset failed" >&2; exit 1; }
sudo "$ESTOP_SH"     || { echo "E-stop clear failed"   >&2; exit 1; }

SERVER_PID=""
WEB_PID=""
if [ "$WITH_SERVER" = 1 ]; then
    echo "  starting server.js (log: $SERVER_LOG)"
    "$NODE_BIN" "$SERVER_JS" > "$SERVER_LOG" 2>&1 &
    SERVER_PID=$!
    sleep 0.5
    kill -0 "$SERVER_PID" 2>/dev/null || {
        echo "ERROR: server.js failed to start — check $SERVER_LOG" >&2; exit 1; }
    python3 "$WEB_SCRIPT" >/dev/null 2>&1 &
    WEB_PID=$!
    echo "  dashboard: http://boneblue-0:$WEB_PORT/bbot_dashboard.html"
else
    echo "  server.js and dashboard skipped (--no-server)"
fi

# ── cleanup, and record how long the run lasted ────────────────────
cleanup() {
    elapsed=$(( $(date +%s) - RUN_ID ))
    stamp "bbot_run END id=$RUN_ID input=$INPUT elapsed=${elapsed}s label='$LABEL'"
    [ -n "$SERVER_PID" ] && { kill "$SERVER_PID" 2>/dev/null; wait "$SERVER_PID" 2>/dev/null; }
    [ -n "$WEB_PID" ]    && { kill "$WEB_PID"    2>/dev/null; wait "$WEB_PID"    2>/dev/null; }
    echo
    echo "  run $RUN_ID ($INPUT) lasted ${elapsed}s = $(( elapsed / 60 ))m $(( elapsed % 60 ))s"
    echo "  If the board died instead of you quitting, that END line never got"
    echo "  written — take the survival time from the START stamp in dmesg and"
    echo "  the timestamp on the first mmc0 error."
}
trap cleanup EXIT INT TERM

echo "  starting balance_bot"
sleep 0.5
sudo TERM="${TERM:-vt100}" "$BALANCE_BOT" $BOT_ARGS
