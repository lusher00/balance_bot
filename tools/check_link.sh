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
# check_link.sh — compile every source in SRCS and report symbols that nothing
# defines, without needing the target's libraries.
#
# Why this exists: `gcc -fsyntax-only` type-checks a translation unit in
# isolation and never links, so it cannot see an undefined reference. A missing
# global definition compiles perfectly in every file and only fails at the link
# on the board — a full sync-and-build round trip away. That happened with
# g_sbus_config: declared extern in balance_bot.h, used by robot_config.c,
# defined in a robot.c that had not been applied yet.
#
# This compiles all objects, takes the union of undefined symbols, subtracts the
# union of defined symbols, and subtracts a list of things the C library and the
# other -l libraries supply. Whatever is left is a real unresolved reference.
#
#   ./tools/check_link.sh              # check
#   ./tools/check_link.sh -v           # also list the external symbols it ignored
#
# Runs on any x86 dev box; it never links, so target-only libraries are fine.

set -uo pipefail
cd "$(dirname "$0")/.."

VERBOSE=0
[ "${1:-}" = "-v" ] && VERBOSE=1
[ "${1:-}" = "--strict" ] && STRICT=1

OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

# ncurses.h and robotcontrol.h may be absent on a dev box. Provide a stub for
# ncurses so display.c compiles; it is only used for symbol extraction, and the
# printf attribute means format bugs still get caught.
mkdir -p "$OUT/stub"
cat > "$OUT/stub/ncurses.h" <<'EOF'
#pragma once
#include <stdio.h>
typedef struct _win_st WINDOW; typedef struct screen SCREEN;
extern WINDOW *stdscr;
#define A_BOLD 1
#define A_DIM 2
#define TRUE 1
#define FALSE 0
#define ACS_HLINE '-'
#define COLOR_BLACK 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_YELLOW 3
#define COLOR_BLUE 4
#define COLOR_CYAN 6
#define COLOR_WHITE 7
#define COLOR_PAIR(n) (n)
int mvprintw(int,int,const char*,...) __attribute__((format(printf,3,4)));
int printw(const char*,...) __attribute__((format(printf,1,2)));
int attron(int); int attroff(int); int endwin(void); int refresh(void);
int curs_set(int); int noecho(void); int cbreak(void); int nodelay(WINDOW*,int);
int getch(void); int has_colors(void); int start_color(void);
int init_pair(short,short,short); int use_default_colors(void);
int mvaddch(int,int,unsigned); int mvhline(int,int,unsigned,int); int getmaxx(WINDOW*);
WINDOW *initscr(void); SCREEN *newterm(const char*,FILE*,FILE*);
SCREEN *set_term(SCREEN*); int delscreen(SCREEN*);
EOF

# Real header wins if the dev box has it.
INC="-Iinclude"
[ -f /usr/include/ncurses.h ] || INC="$INC -I$OUT/stub"

SRCS=$(grep -oE 'src/[a-z_]+\.c' Makefile | sort -u)
[ -z "$SRCS" ] && { echo "no sources found in Makefile SRCS"; exit 1; }

echo "Compiling $(echo "$SRCS" | wc -w) objects..."
fail=0
warned=0
for f in $SRCS; do
    if ! gcc -Wall -Wextra -O2 $INC -c "$f" -o "$OUT/$(basename "${f%.c}").o" 2>"$OUT/err"; then
        echo "  COMPILE FAIL: $f"
        grep -E 'error:' "$OUT/err" | head -5 | sed 's/^/      /'
        fail=1
    elif grep -q 'warning:' "$OUT/err"; then
        # Warnings are reported but do NOT fail the run. The tree carries some
        # pre-existing -Wunused-result noise from rc_compat.h, and failing on it
        # would make this tool useless for its actual job, which is finding
        # unresolved symbols. Use --strict when you want warnings to fail.
        warned=$((warned + 1))
        echo "  warnings in $f:"
        grep -E 'warning:' "$OUT/err" | head -5 | sed 's/^/      /'
    fi
done
[ "$fail" -ne 0 ] && { echo; echo "FAILED: compile errors above."; exit 1; }
if [ "$warned" -ne 0 ]; then
    echo "  compiled with warnings in $warned file(s)"
    [ "${STRICT:-0}" = "1" ] && { echo "FAILED: --strict and warnings present."; exit 1; }
else
    echo "  all compiled clean"
fi

nm -g -u "$OUT"/*.o 2>/dev/null | awk '/^ *U /{print $2}' | sed 's/@.*//' | sort -u > "$OUT/undef"
nm -g --defined-only "$OUT"/*.o 2>/dev/null | awk '{print $3}' | sort -u > "$OUT/def"
comm -23 "$OUT/undef" "$OUT/def" > "$OUT/external"

# Symbols supplied by libc/libm/libpthread/libncurses/librobotcontrol. Anything
# matching is expected to resolve at link time on the target.
EXTERNAL_RE='^(rc_|__|_)|^(mem|str|wcs)|^(sn?printf|v?f?printf|f?scanf|sscanf|put(s|char|c)|fput(s|c)|fget(s|c)|getchar)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(malloc|calloc|realloc|free|exit|abort|atexit|system|getenv|setenv)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(open|close|read|write|lseek|access|unlink|rename|remove|dup|dup2|pipe|fcntl|ioctl|stat|fstat|lstat|mkdir|chmod|readlink|realpath|getcwd|glob|globfree|opendir|readdir|closedir)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(fopen|fclose|fdopen|freopen|fflush|fread|fwrite|fseek|ftell|fileno|fsync|popen|pclose|stdin|stdout|stderr)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(atoi|atof|strtol|strtoul|strtof|strtod|qsort|bsearch|abs|labs|rand|srand)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(is|to)(space|digit|alpha|alnum|upper|lower|print|punct)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(time|clock_gettime|gettimeofday|localtime|localtime_r|gmtime|strftime|mktime|difftime|usleep|sleep|nanosleep)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(socket|bind|listen|accept|connect|send|sendto|recv|recvfrom|setsockopt|getsockname|shutdown|select|poll|inet_addr|inet_ntoa|htons|ntohs)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(signal|sigaction|sigemptyset|sigaddset|sigprocmask|kill|raise|fork|execvp|execl|waitpid|getpid|getuid|geteuid|setuid)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^pthread_|^sem_'
EXTERNAL_RE="$EXTERNAL_RE"'|^(tcgetattr|tcsetattr|tcflush|tcdrain|cfsetispeed|cfsetospeed|cfmakeraw)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(fabsf?|sqrtf?|cbrtf?|sinf?|cosf?|tanf?|asinf?|acosf?|atanf?|atan2f?|sinhf?|coshf?|tanhf?|expf?|exp2f?|logf?|log2f?|log10f?|powf?|hypotf?|floorf?|ceilf?|roundf?|truncf?|fmodf?|fmaxf?|fminf?|copysignf?|isnanf?|isinff?|nanf?)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(getopt|getopt_long|optarg|optind|opterr|optopt)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(strerror|strerror_r|perror|errno)$'
EXTERNAL_RE="$EXTERNAL_RE"'|^(mvprintw|printw|mvaddch|mvhline|getmaxx|attron|attroff|initscr|endwin|refresh|curs_set|noecho|cbreak|nodelay|getch|has_colors|start_color|init_pair|use_default_colors|stdscr|newterm|set_term|delscreen|clear|erase|move|box|wrefresh|keypad|timeout|mvaddstr|addstr)$'

grep -vE "$EXTERNAL_RE" "$OUT/external" > "$OUT/suspect" || true

if [ "$VERBOSE" -eq 1 ]; then
    echo
    echo "External symbols assumed provided by libraries:"
    grep -E "$EXTERNAL_RE" "$OUT/external" | sed 's/^/    /'
fi

echo
if [ -s "$OUT/suspect" ]; then
    echo "UNRESOLVED — nothing in the object set defines these:"
    sed 's/^/    /' "$OUT/suspect"
    echo
    echo "If any is a project global, it needs a definition (not just an extern"
    echo "declaration in a header). If it is a library symbol, add it to"
    echo "EXTERNAL_RE in this script."
    exit 1
fi
echo "OK — every internal symbol is defined by some object."
