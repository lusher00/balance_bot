#!/usr/bin/env bash
# Merge config/overrides.conf into the bot's live robot.conf and restart the
# service. The live file stays the only copy that matters; this only touches
# the keys named in the overrides and backs the file up first.
#
#   tools/push_conf.sh            merge + restart
#   tools/push_conf.sh --dry      merge into a scratch copy on the bot, show the result, change nothing
set -euo pipefail
BOT=${BOT:-boneblue-0}
DIR=$(cd "$(dirname "$0")/.." && pwd)
OV="$DIR/config/overrides.conf"
[ -f "$OV" ] || { echo "no $OV"; exit 1; }
scp -q "$DIR/tools/conf_patch.py" "$OV" "$BOT:/tmp/"
if [ "${1:-}" = "--dry" ]; then
  ssh "$BOT" 'cp ~/balance_bot/robot.conf /tmp/robot.conf.dry && python3 /tmp/conf_patch.py /tmp/robot.conf.dry /tmp/overrides.conf && echo && diff -u ~/balance_bot/robot.conf /tmp/robot.conf.dry || true'
  exit 0
fi
ssh "$BOT" 'python3 /tmp/conf_patch.py ~/balance_bot/robot.conf /tmp/overrides.conf && sudo systemctl restart balance_bot && sleep 2 && systemctl is-active balance_bot'
