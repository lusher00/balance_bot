# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# This file is part of balance_bot, licensed under the PolyForm
# Noncommercial License 1.0.0. You may use, study, modify, and share
# it for any noncommercial purpose. Commercial use requires a separate
# license from the author -- contact ryan.lush@gmail.com.
# Full license text: see the LICENSE file in the project root, or
# https://polyformproject.org/licenses/noncommercial/1.0.0/

# bot_aliases.sh — balance_bot service control, sourced from ~/.bashrc
#
# THE SPLIT. This file holds what is specific to THIS PROJECT: the services,
# their logs, and the project's own tools. Everything about having a usable
# shell -- history, prompt, ls, directory sizes, git, i2c, dmesg -- lives in
# ~/.bashrc, because it is worth having on a board with no balance_bot on it.
#
# WHY A SPLIT IS SAFE HERE, when the previous two-file arrangement was not.
# The old bug was NOT that there were two files. It was that both defined the
# SAME NAMES -- bots, sbot and rbot appeared in each -- and whichever was
# sourced last won silently. The file you would read to find out what sbot did
# described the definition that never took effect.
#
# So the rule is: no name may be defined in both files.
# config/install_bashrc.sh checks it and refuses to install on a collision, so
# this cannot rot back into the old state.
#
# bhelp reads BOTH files, so everything still appears in one menu.
#
# This is sourced from the repo, so editing it here takes effect on the next
# shell -- no reinstall needed.

alias bb='cd ~/balance_bot'   #: cd to the project

#:: Balance Bot services
# -n 0 --no-pager on every status: `systemctl status` tails the last 10 journal
# lines by default, which for these services is a wall of application output
# that buries the one thing you asked for -- whether it is running. Use the
# *log aliases below when you want the log.
# One family per service, plus an "all" group. Consistent shape:
#   <svc>s   status      s<svc>   stop        r<svc>   restart
# bbb_oled is included in the all-group. It used to be missing from it, so
# 'sbots' left the display running and it kept showing a service that was no
# longer there.
BB_SERVICES='balance_bot balance_bot_server batt_monitor bbb_oled'

alias botss="systemctl status -n 0 --no-pager $BB_SERVICES"           #: status ALL bot services
alias sbots="sudo systemctl stop $BB_SERVICES"        #: stop ALL bot services
alias rbots="sudo systemctl restart $BB_SERVICES"     #: restart ALL bot services

alias bots='systemctl status -n 0 --no-pager balance_bot'             #: status balance_bot
alias sbot='sudo systemctl stop balance_bot'          #: stop balance_bot
alias rbot='sudo systemctl restart balance_bot'       #: restart balance_bot

alias srvs='systemctl status -n 0 --no-pager balance_bot_server'      #: status the websocket bridge
alias ssrv='sudo systemctl stop balance_bot_server'   #: stop the websocket bridge
alias rsrv='sudo systemctl restart balance_bot_server' #: restart the websocket bridge

alias batts='systemctl status -n 0 --no-pager batt_monitor'           #: status battery monitor
alias sbatt='sudo systemctl stop batt_monitor'        #: stop battery monitor
alias rbatt='sudo systemctl restart batt_monitor'     #: restart battery monitor

alias oleds='systemctl status -n 0 --no-pager bbb_oled'               #: status OLED display
alias soled='sudo systemctl stop bbb_oled'            #: stop OLED display
alias roled='sudo systemctl restart bbb_oled'         #: restart OLED display

alias watchs='systemctl status -n 0 --no-pager bbot-watch'            #: status system recorder
alias swatch='sudo systemctl stop bbot-watch'         #: stop system recorder
alias rwatch='sudo systemctl restart bbot-watch'      #: restart system recorder

#:: Bot logs
alias botlog='journalctl -u balance_bot -f'           #: follow balance_bot
alias serverlog='journalctl -u balance_bot_server -f' #: follow the bridge
alias battlog='journalctl -u batt_monitor -f'         #: follow battery monitor
alias oledlog='journalctl -u bbb_oled -f'             #: follow the OLED display
alias watchlog='journalctl -u bbot-watch -f'          #: follow the system recorder

#:: Balance Bot tools
alias btune='python3 ~/balance_bot/tools/analyze_tune.py'    #: analyse a recorded tuning run
alias brate='~/balance_bot/tools/sock_rate.py'               #: telemetry rate off the unix socket, per type
alias bwatch='sudo ~/balance_bot/tools/bbot_watch.py'        #: system recorder (Ctrl-C to stop)
alias bpost='~/balance_bot/tools/bbot_watch.py --postmortem' #: what happened before the last lockup
alias bweb='python3 ~/balance_bot/web/serve_web.py'          #: serve the dashboard on :8888
alias bestop='python3 ~/balance_bot/roboclaw_reset.py && ~/balance_bot/estop_clear.sh'  #: clear a latched RoboClaw e-stop
