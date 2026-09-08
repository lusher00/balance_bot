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
# install_bashrc.sh — put config/bashrc in place as ~/.bashrc.
#
#   ./config/install_bashrc.sh              show what would change
#   ./config/install_bashrc.sh --apply      install it, keeping a timestamped backup
#   ./config/install_bashrc.sh --file=X     install a specific file
#
# Why a versioned file rather than editing ~/.bashrc directly: ~/.bashrc is not
# in the repo, so every alias in it was one reflash away from being gone, and
# the previous arrangement (this file, plus config/aliases.sh, plus an unsourced
# tools/ls_aliases.sh) meant three places could disagree -- and did. One file,
# in git, installed here.
#
# Your machine-specific bits belong in ~/.bash_aliases, which this never
# touches and config/bashrc sources at the end.

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
        "" "${BASH_SOURCE[0]}" >&2
    return 1
fi

set -u

# Per-machine source files. The suffix names the target, so a Mac copy can sit
# beside the bot's without either overwriting the other:
#
#     bashrc.bone   the BeagleBone
#     bashrc.mac    a Mac, if you add one later
#
# Picked automatically from the OS; override with --file.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "$(uname -s)" in
    Darwin) DEFAULT_SRC="$REPO/bashrc.mac" ;;
    *)      DEFAULT_SRC="$REPO/bashrc.bone" ;;
esac
# Fall back to whichever exists, so this still works on a machine with only one.
[ -f "$DEFAULT_SRC" ] || DEFAULT_SRC="$REPO/bashrc.bone"
[ -f "$DEFAULT_SRC" ] || DEFAULT_SRC="$REPO/bashrc.mac"

SRC="$DEFAULT_SRC"
DST="$HOME/.bashrc"
APPLY=0
for arg in "${@:-}"; do
    case "$arg" in
        --apply) APPLY=1 ;;
        --file=*) SRC="${arg#--file=}" ;;
        "") ;;
        *) printf "unknown option: %s\n" "$arg" >&2; exit 1 ;;
    esac
done

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33m==> %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[31m==> %s\033[0m\n' "$*" >&2; exit 1; }

[ -f "$SRC" ] || die "missing $SRC  (expected the machine's bashrc in $REPO)"
say "source: $SRC"

# Never install something that will not parse -- a broken ~/.bashrc greets you
# with a syntax error on every new shell, including the one you would use to fix it.
bash -n "$SRC" || die "$SRC has a syntax error; refusing to install"
say "syntax check passed"

# ── collision check ──────────────────────────────────────────────────────
# bashrc.bone sources ~/.bash_aliases at the very end, and any legacy
# config/aliases.sh may still be lying around on the bot. Whatever is sourced
# LAST wins, silently -- which is exactly how sbot ended up stopping one service
# while the file you would read to find out said it stopped three.
#
# So: list every name bashrc.bone defines, list every name the other files
# define, and report the overlap. Names, not contents, because a collision is a
# collision even when the two definitions happen to agree today.
names_in() {
    [ -f "$1" ] || return 0
    grep -hoE '^[[:space:]]*alias[[:space:]]+[A-Za-z0-9_.]+=' "$1" 2>/dev/null |
        sed 's/.*alias[[:space:]]*//; s/=$//'
    grep -hoE '^[A-Za-z0-9_]+\(\)' "$1" 2>/dev/null | sed 's/()//'
}

MINE="$(mktemp)"; THEIRS="$(mktemp)"
trap 'rm -f "$MINE" "$THEIRS"' EXIT
names_in "$SRC" | sort -u > "$MINE"

# The two project files must never define the same name. This is the rule that
# makes the split safe -- see the header of config/bot_aliases.sh.
BOT_ALIASES="$REPO/config/bot_aliases.sh"
if [ -f "$BOT_ALIASES" ]; then
    BOTN="$(mktemp)"
    names_in "$BOT_ALIASES" | sort -u > "$BOTN"
    OVERLAP="$(comm -12 "$MINE" "$BOTN" | tr '\n' ' ')"
    rm -f "$BOTN"
    if [ -n "${OVERLAP// /}" ]; then
        die "$(basename "$SRC") and config/bot_aliases.sh both define: $OVERLAP
       That is the exact bug the split exists to prevent -- one file wins
       silently and the other becomes a lie. Remove the duplicates and re-run."
    fi
    say "no overlap between $(basename "$SRC") and bot_aliases.sh"
fi

LEGACY=""
for cand in "$HOME/.bash_aliases" "$REPO/config/aliases.sh" "$REPO/tools/ls_aliases.sh"; do
    [ -f "$cand" ] || continue
    LEGACY="$LEGACY $cand"
    names_in "$cand" >> "$THEIRS"
done

if [ -n "$LEGACY" ]; then
    sort -u "$THEIRS" -o "$THEIRS"
    CLASH="$(comm -12 "$MINE" "$THEIRS" | tr '\n' ' ')"
    if [ -n "${CLASH// /}" ]; then
        warn "these names are defined BOTH in $(basename "$SRC") and elsewhere:"
        echo "      $CLASH"
        echo "  Defined in:$LEGACY"
        echo "  Those are sourced AFTER bashrc, so they win -- and bhelp will"
        echo "  describe the definition that did NOT take effect."
        echo
        echo "  Either delete the duplicates from those files, or move the whole"
        echo "  file aside if it is fully superseded:"
        for l in $LEGACY; do echo "      mv $l $l.superseded"; done
        echo
    else
        say "no name collisions with$LEGACY"
    fi
fi

if [ -f "$DST" ] && cmp -s "$SRC" "$DST"; then
    say "$DST is already identical — nothing to do"
    exit 0
fi

if [ -f "$DST" ]; then
    say "differences against the current $DST:"
    diff -u "$DST" "$SRC" | sed -n '1,60p' || true
    echo
    n=$(diff "$DST" "$SRC" | grep -c '^[<>]' || true)
    say "$n changed line(s) in total"
else
    say "no existing $DST — this will create it"
fi

if [ "$APPLY" -eq 0 ]; then
    echo
    say "dry run — nothing written. Re-run with --apply"
    exit 0
fi

if [ -f "$DST" ]; then
    BAK="$DST.bak-$(date +%Y%m%d-%H%M%S)"
    cp -p "$DST" "$BAK" || die "could not back up $DST"
    say "backed up to $BAK"
fi

cp "$SRC" "$DST" || die "could not write $DST"
say "installed $DST"

# Verify the installed copy in a real shell rather than trusting the copy.
if bash -ic 'declare -F bhelp >/dev/null && alias botss >/dev/null' 2>/dev/null; then
    say "verified: bhelp and the service aliases load in a new shell"
else
    warn "installed, but a new shell did not expose bhelp/botss -- check for an"
    warn "early 'return' or an error in $DST"
fi

echo
say "Run 'exec bash' or open a new terminal, then:"
echo "     bhelp        project commands, grouped"
echo "     ahelp        every alias and function"
