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
# git_cleanup.sh -- take the 1.35 GB SD image back out of git history.
#
# THE PROBLEM
#
#   Commit 93ea304 ("added linux image", 2026-08-27) added
#   bbb-sd-2026-08-27.img.gz -- 1,355,859,661 bytes -- to the repository.
#   The file has since been deleted from the working tree, but deleting a
#   file does not remove it from the commit that added it. So:
#
#     * .git is 1.3 GB (pack alone is 1.19 GiB)
#     * `git push` to GitHub WILL FAIL and cannot be made to succeed by
#       retrying: GitHub hard-rejects any blob over 100 MB
#     * every future clone pays for the image forever
#
#   main is currently 1 commit ahead of origin/main, and that commit is the
#   one carrying the image. Nothing has been pushed yet, which is the only
#   reason this is cheap to fix -- no one else has the bad history.
#
# READ THIS BEFORE RUNNING
#
#   The image exists in exactly ONE place on this machine: inside .git.
#   It is NOT in the working tree. Step 1 below extracts it to a real file
#   BEFORE anything destructive happens, because you asked for an archive
#   copy of the card and this is currently it. If you already have that
#   archive somewhere safe, step 1 is still harmless -- it just costs disk.
#
#   Only the tip commit is rewritten. Its message, author and date are
#   preserved; every other commit is untouched. Your 64 uncommitted working
#   files are not touched at all -- this only rewrites what is already
#   committed.
#
# USAGE
#
#   tools/git_cleanup.sh              # dry run: report only, change nothing
#   tools/git_cleanup.sh --go         # actually do it
#
# AFTER IT FINISHES
#
#   git log --stat -1                 # confirm the image is gone from the commit
#   du -sh .git                       # should be a few MB, not 1.3 GB
#   git push                          # should now succeed
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BAD_COMMIT="93ea304"
IMAGE="bbb-sd-2026-08-27.img.gz"
ARCHIVE_DIR="${ARCHIVE_DIR:-$HOME/bbot-card-archive}"

GO=0
[[ "${1:-}" == "--go" ]] && GO=1

say()  { printf '%s\n' "$*"; }
step() { printf '\n== %s\n' "$*"; }

# ── preflight ───────────────────────────────────────────────────────────────
step "Preflight"

if ! git rev-parse --git-dir >/dev/null 2>&1; then
    say "Not a git repository: $REPO_ROOT"; exit 1
fi

# A stale .git/index.lock is left behind by a git process that died -- often an
# editor's git integration. Every index write fails until it is removed, which
# includes `git rm --cached` in step 2 but NOT `git reset --soft` in front of it:
# --soft moves HEAD and never touches the index. So without this check the
# script gets exactly one command into the rewrite and stops there, half done.
if [[ -e .git/index.lock ]]; then
    say "A git lock file is in the way:"
    say "    $(ls -l .git/index.lock)"
    say ""
    say "Every index write will fail until it is gone. Check nothing is actually"
    say "running first (an open editor, a stuck 'git commit'):"
    say "    ps ax | grep '[g]it '"
    say "If that is empty -- and an old timestamp above means it is -- remove it:"
    say "    rm -f $REPO_ROOT/.git/index.lock"
    exit 1
fi

if ! git cat-file -e "${BAD_COMMIT}:${IMAGE}" 2>/dev/null; then
    say "The image is not in ${BAD_COMMIT}. Either this already ran, or the"
    say "commit hash moved. Nothing to do -- check 'git log --oneline' first."
    exit 0
fi

HEAD_SHA="$(git rev-parse HEAD)"
BAD_SHA="$(git rev-parse "$BAD_COMMIT")"
BAD_PARENT="$(git rev-parse "${BAD_COMMIT}^")"

# Three states, not two. The middle one exists because step 2 is three commands
# and only the first survives a locked index: `git reset --soft` moves HEAD
# without touching the index, then `git rm --cached` fails. That leaves HEAD on
# the parent with the bad commit's whole tree -- image included -- still staged.
# Refusing to run there would be wrong: the work is half done and the rest is
# the same two commands.
RESUME=0
if [[ "$HEAD_SHA" == "$BAD_SHA" ]]; then
    :
elif [[ "$HEAD_SHA" == "$BAD_PARENT" ]] && git ls-files --cached --error-unmatch "$IMAGE" >/dev/null 2>&1; then
    RESUME=1
    say "RESUMING: HEAD is already on ${BAD_COMMIT}'s parent and the image is still"
    say "staged -- a previous run got through the reset and no further. Skipping"
    say "the reset; the remaining work is unchanged."
else
    say "HEAD is $HEAD_SHA but the bad commit is $BAD_SHA."
    say "This script only handles the case where the bad commit is the TIP,"
    say "or a run that stopped part way through the rewrite."
    say "Commits have landed on top of it since. Stop and re-plan -- you now"
    say "need a range rewrite (git filter-repo), not this."
    exit 1
fi

UPSTREAM="$(git rev-parse --abbrev-ref '@{upstream}' 2>/dev/null || echo none)"
AHEAD="$(git rev-list --count '@{upstream}..HEAD' 2>/dev/null || echo '?')"
say "repo          $REPO_ROOT"
say "HEAD          $(git log --oneline -1)"
say "upstream      $UPSTREAM (ahead by $AHEAD)"
say ".git size     $(du -sh .git | cut -f1)"
say "image blob    $(git cat-file -s "${BAD_COMMIT}:${IMAGE}") bytes"

if [[ -n "$(git rev-list "@{upstream}..HEAD" --not --all 2>/dev/null; true)" ]]; then :; fi

if [[ $RESUME -eq 0 && "$AHEAD" != "1" && "$AHEAD" != "?" ]]; then
    say ""
    say "WARNING: expected to be exactly 1 commit ahead of upstream, but it is $AHEAD."
    say "Rewriting the tip is still safe, but confirm nothing else needs preserving."
fi

if [[ $GO -eq 0 ]]; then
    say ""
    say "DRY RUN -- nothing changed. Re-run with --go to proceed."
    say "It will:"
    say "  1. extract $IMAGE to $ARCHIVE_DIR/"
    say "  2. rewrite $BAD_COMMIT to drop the image (message/author/date kept)"
    say "  3. expire the reflog and garbage-collect"
    exit 0
fi

# ── 1. rescue the image ─────────────────────────────────────────────────────
step "1/3  Extracting the image before anything destructive"

mkdir -p "$ARCHIVE_DIR"
OUT="$ARCHIVE_DIR/$IMAGE"

if [[ -f "$OUT" ]]; then
    say "Already present: $OUT ($(du -h "$OUT" | cut -f1))"
    say "Leaving it alone. Delete it yourself if you want a fresh extract."
else
    say "Writing $OUT ..."
    git cat-file blob "${BAD_COMMIT}:${IMAGE}" > "$OUT"
    say "Wrote $(du -h "$OUT" | cut -f1)"
fi

EXPECT="$(git cat-file -s "${BAD_COMMIT}:${IMAGE}")"
ACTUAL="$(wc -c < "$OUT" | tr -d ' ')"
if [[ "$EXPECT" != "$ACTUAL" ]]; then
    say "SIZE MISMATCH: expected $EXPECT bytes, got $ACTUAL. Stopping before"
    say "anything destructive happens. Do not proceed."
    exit 1
fi
say "Size verified: $ACTUAL bytes."
say "Sanity-check the archive yourself before trusting it:"
say "    gzip -t \"$OUT\" && echo 'gzip stream OK'"

# ── 2. rewrite the tip commit ───────────────────────────────────────────────
step "2/3  Rewriting $BAD_COMMIT without the image"

# Index is expected clean; the working tree is not touched by any of this.
if [[ $RESUME -eq 0 ]]; then
    git reset --soft HEAD~1
else
    say "(reset already done by the previous run)"
fi
git rm --cached --ignore-unmatch "$IMAGE" >/dev/null
git commit -C "$BAD_SHA" --no-verify >/dev/null
say "New tip: $(git log --oneline -1)"

if git cat-file -e "HEAD:${IMAGE}" 2>/dev/null; then
    say "The image is STILL in HEAD. Something went wrong -- stop and inspect."
    exit 1
fi
say "Confirmed: the image is no longer in the commit."

# ── 3. reclaim the space ────────────────────────────────────────────────────
step "3/3  Expiring the reflog and garbage-collecting"

say "(the old commit is still reachable via the reflog until this runs)"
git reflog expire --expire=now --expire-unreachable=now --all
git gc --prune=now
say ".git size now: $(du -sh .git | cut -f1)"

step "Done"
say "Archive:  $OUT"
say "Verify:   git log --stat -1"
say "Then:     git push"
