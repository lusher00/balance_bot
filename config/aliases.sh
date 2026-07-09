# ── balance_bot aliases ───────────────────────────────────────────────
alias ll='ls -l'
alias la='ls -lA'
alias l='ls -CF'
alias bots='systemctl status balance_bot balance_bot_server batt_monitor'  # status all
alias sbot='sudo systemctl stop balance_bot balance_bot_server batt_monitor' # stop all
alias rbot='sudo systemctl restart balance_bot balance_bot_server'           # restart all
alias zbot='sudo systemctl status balance_bot'                               # bot status
alias zsrv='sudo systemctl status balance_bot_server'                        # server status
alias batt='sudo systemctl status batt_monitor'                              # batt status
alias rbatt='sudo systemctl restart batt_monitor'                            # batt restart
alias sbatt='sudo systemctl stop batt_monitor'                               # batt stop
alias oled='sudo systemctl status bbb-oled'                                  # oled status
alias roled='sudo systemctl restart bbb-oled'                                # oled restart
alias soled='sudo systemctl stop bbb-oled'                                   # oled stop

# ── help ──────────────────────────────────────────────────────────────
bhelp() {
    echo ""
    grep "^alias" ~/balance_bot/config/aliases.sh | \
    sed "s/alias //;s/=.* #/\t#/" | \
    awk -F'\t' '{printf "  %-12s %s\n", $1, $2}'
    echo ""
}
