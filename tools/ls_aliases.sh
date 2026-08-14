# shellcheck shell=bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# ls aliases — sort by type and by modified time, with colour and detail.
#
# Source it from ~/.zshrc (Mac) or ~/.bashrc (BeagleBone):
#
#     [ -f ~/balance_bot/tools/ls_aliases.sh ] && . ~/balance_bot/tools/ls_aliases.sh
#
# Then `exec $SHELL` or open a new terminal.
#
# The same alias names work on all three back ends, so muscle memory carries
# between your Mac and the bot:
#
#     eza      if installed — best output: git status, tree, headers
#     GNU ls   the BeagleBone, and Mac with coreutils installed
#     BSD ls   stock macOS
#
# BSD ls cannot group directories first or sort by extension, so on stock macOS
# those two fall back to something close rather than pretending. Install eza
# (`brew install eza`) or coreutils (`brew install coreutils`) to get the full
# set — the script picks it up automatically.
#
# ── the aliases ──────────────────────────────────────────────────────
#   l      brief, columns
#   ll     long, human sizes, directories first
#   la     ll plus dotfiles
#   lt     by modified time, NEWEST FIRST
#   ltr    by modified time, NEWEST LAST   (good for tailing a log dir)
#   lx     grouped by extension — "sort by file type"
#   lz     by size, largest first
#   ld     directories only
#   lf     files only
#   lg     long + git status            (eza only)
#   lr     recursive tree, 2 deep
#   lnew   the 15 most recently modified files, recursively
#   lbig   the 15 largest files, recursively

# ── back end detection ───────────────────────────────────────────────
_bb_ls_backend() {
    if command -v eza >/dev/null 2>&1; then
        echo eza
    elif command -v gls >/dev/null 2>&1; then
        echo gnu-g          # coreutils on macOS installs GNU ls as `gls`
    elif ls --version >/dev/null 2>&1; then
        echo gnu
    else
        echo bsd
    fi
}

case "$(_bb_ls_backend)" in

eza)
    # Richest option. --group-directories-first and --git are the wins.
    _EZA='eza --color=auto --icons=auto --group-directories-first'
    alias l="$_EZA"
    alias ll="$_EZA --long --header --group --time-style=long-iso"
    alias la="$_EZA --long --header --group --time-style=long-iso --all"
    alias lt="$_EZA --long --header --time-style=long-iso --sort=modified --reverse"
    alias ltr="$_EZA --long --header --time-style=long-iso --sort=modified"
    alias lx="$_EZA --long --header --time-style=long-iso --sort=extension"
    alias lz="$_EZA --long --header --time-style=long-iso --sort=size --reverse"
    alias ld="$_EZA --long --header --only-dirs"
    alias lf="$_EZA --long --header --only-files"
    alias lg="$_EZA --long --header --git --git-repos --time-style=long-iso"
    alias lr="$_EZA --tree --level=2"
    ;;

gnu|gnu-g)
    # `gls` on macOS+coreutils, plain `ls` on Linux.
    if command -v gls >/dev/null 2>&1; then _LS=gls; else _LS=ls; fi
    _GNU="$_LS --color=auto --group-directories-first -h --time-style=long-iso"
    alias l="$_LS --color=auto --group-directories-first -CF"
    alias ll="$_GNU -lF"
    alias la="$_GNU -lAF"
    alias lt="$_GNU -lAFt"          # -t newest first
    alias ltr="$_GNU -lAFtr"        # oldest first, newest at the bottom
    alias lx="$_GNU -lAFX"          # -X sorts by extension
    alias lz="$_GNU -lAFS"          # -S largest first
    alias ld="$_LS --color=auto -d */"
    alias lf="$_GNU -lF | grep -v '^d'"
    alias lg='ll'                   # no git integration without eza
    alias lr="$_LS --color=auto -R --group-directories-first -F"
    unset _GNU
    ;;

bsd)
    # Stock macOS. -G is colour; there is no directory grouping or -X.
    export CLICOLOR=1
    alias l='ls -CFG'
    alias ll='ls -lhFG'
    alias la='ls -lhAFG'
    alias lt='ls -lhAFGt'
    alias ltr='ls -lhAFGtr'
    # No -X on BSD ls, so approximate "by type" by sorting the long listing on
    # the filename extension. Loses colour (it is piped), which is the honest
    # trade rather than silently doing nothing.
    lx() { ls -lhAF "$@" | awk 'NR==1{print;next}{print}' | sort -t. -k2; }
    alias lz='ls -lhAFGS'
    alias ld='ls -dG */'
    alias lf="ls -lhFG | grep -v '^d'"
    alias lg='ll'
    alias lr='ls -RCFG'
    ;;
esac

# ── recursive helpers, back-end independent ──────────────────────────

# 15 most recently modified files below here. Useful in a log directory.
#
# GNU find has -printf, BSD find does not. Detect up front rather than running
# the GNU form and falling back on failure -- that emits partial output first,
# so you get half a listing followed by a second, complete one.
lnew() {
    local n="${1:-15}"
    if find . -maxdepth 0 -printf '' >/dev/null 2>&1; then
        find . -type f -not -path '*/.git/*' -not -path '*/node_modules/*' \
            -printf '%T@|%TY-%Tm-%Td %TH:%TM|%8s|%p\n' 2>/dev/null |
            sort -rn | head -n "$n" |
            awk -F'|' '{printf "  %s  %8s  %s\n", $2, $3, $4}'
    else
        find . -type f -not -path '*/.git/*' -not -path '*/node_modules/*' \
            -print0 2>/dev/null |
            xargs -0 stat -f '%m|%Sm|%z|%N' -t '%Y-%m-%d %H:%M' 2>/dev/null |
            sort -rn | head -n "$n" |
            awk -F'|' '{printf "  %s  %8s  %s\n", $2, $3, $4}'
    fi
}

# 15 largest files below here.
lbig() {
    local n="${1:-15}"
    find . -type f -not -path '*/.git/*' -not -path '*/node_modules/*' \
        -exec du -h {} + 2>/dev/null | sort -rh | head -n "$n"
}

# ── colour ───────────────────────────────────────────────────────────
# GNU LS_COLORS: brighter, and gives archives/media/code distinct colours so
# "sort by type" is legible at a glance rather than needing the extension read.
if [ -z "${LS_COLORS:-}" ]; then
    export LS_COLORS='di=1;34:ln=1;36:so=1;35:pi=33:ex=1;32:bd=1;33:cd=1;33:su=37;41:sg=30;43:tw=30;42:ow=34;42:\
*.tar=1;31:*.tgz=1;31:*.zip=1;31:*.gz=1;31:*.bz2=1;31:*.xz=1;31:*.7z=1;31:\
*.jpg=1;35:*.jpeg=1;35:*.png=1;35:*.gif=1;35:*.svg=1;35:*.mp4=1;35:*.mov=1;35:\
*.c=1;33:*.h=1;33:*.py=1;33:*.sh=1;33:*.js=1;33:*.html=1;33:*.json=0;33:\
*.csv=1;36:*.log=0;36:*.md=0;37:*.conf=0;32:*.service=0;32:*.dtbo=0;32:*.dts=0;32'
fi
# BSD equivalent: dir, symlink, socket, pipe, exec, block, char, setuid, setgid, sticky
export LSCOLORS='ExGxFxdaCxDaDahbadacec'

unset -f _bb_ls_backend
