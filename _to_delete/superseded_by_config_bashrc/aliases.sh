# ── balance_bot aliases ───────────────────────────────────────────────
alias botss='systemctl status balance_bot balance_bot_server batt_monitor' 		 # status all
alias sbots='sudo systemctl stop balance_bot balance_bot_server batt_monitor' 	  	# stop all
alias rbots='sudo systemctl restart balance_bot balance_bot_server batt_monitor'           # restart all

alias sbot='sudo systemctl stop balance_bot'                               # bot status
alias bots='systemctl status balance_bot'
alias rbot='sudo systemctl restart balance_bot'

alias ssrv='sudo systemctl stop balance_bot_server'                        # server status
alias srvs='systemctl status balance_bot_server'
alias rsrv='sudo systemctl restart balance_bot_server'

alias sbatt='sudo systemctl stop batt_monitor'                              # batt status
alias batts='systemctl status batt_monitor'                            # batt restart
alias rbatt='sudo systemctl restart batt_monitor'
 
alias soled='sudo systemctl stop bbb-oled'                                  # oled status
alias oleds='systemctl status bbb-oled'                                # oled restart
alias roled='sudo systemctl restart bbb-oled'                                   # oled stop

# ── help ──────────────────────────────────────────────────────────────
bhelp() {
    echo ""
    grep "^alias" ~/balance_bot/config/aliases.sh | \
    sed "s/alias //;s/=.* #/\t#/" | \
    awk -F'\t' '{printf "  %-12s %s\n", $1, $2}'
    echo ""
}
