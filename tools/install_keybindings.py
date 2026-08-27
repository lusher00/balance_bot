#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
install_keybindings.py — put the balance_bot task shortcuts into VS Code.

WHY THIS EXISTS. VS Code has no workspace-level keybindings. The file in this
repo, .vscode/keybindings.example.json, is inert — a template. The one VS Code
actually reads is a single personal file outside the repo:

    ~/Library/Application Support/Code/User/keybindings.json      (macOS)

So the shortcuts have to be copied there by hand, and the "args" in each entry
has to match a task label in .vscode/tasks.json character for character. When it
does not, VS Code reports nothing at all — it silently opens the task picker,
which looks exactly like "my hotkey stopped working and now it asks me to pick".
Renaming a task therefore breaks shortcuts with no error, which is what this
script is here to stop.

    ./tools/install_keybindings.py            # show what would change
    ./tools/install_keybindings.py --apply    # do it (backs up first)

Safe to re-run. It replaces only the entries it owns — anything else in your
keybindings.json is left exactly as it was.
"""
import argparse, json, os, re, shutil, sys, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
EXAMPLE = os.path.join(REPO, ".vscode", "keybindings.example.json")
TASKS = os.path.join(REPO, ".vscode", "tasks.json")

# Every place VS Code (and its forks) keep the user keybindings file.
CANDIDATES = [
    "~/Library/Application Support/Code/User/keybindings.json",
    "~/Library/Application Support/Code - Insiders/User/keybindings.json",
    "~/Library/Application Support/VSCodium/User/keybindings.json",
    "~/Library/Application Support/Cursor/User/keybindings.json",
    "~/.config/Code/User/keybindings.json",
    "~/.config/Code - Insiders/User/keybindings.json",
]

RUNTASK = "workbench.action.tasks.runTask"


def strip_jsonc(text):
    """Remove // and /* */ comments and trailing commas, respecting strings.

    VS Code writes this file with comments by default, so json.loads() fails on
    a perfectly normal one. Character-by-character rather than a regex because a
    URL inside a string ("https://...") must not be treated as a comment.
    """
    out, i, n = [], 0, len(text)
    in_str = in_line = in_block = False
    while i < n:
        c, nxt = text[i], text[i + 1] if i + 1 < n else ""
        if in_line:
            if c == "\n":
                in_line = False
                out.append(c)
        elif in_block:
            if c == "*" and nxt == "/":
                in_block = False
                i += 1
        elif in_str:
            out.append(c)
            if c == "\\":
                if i + 1 < n:
                    out.append(nxt)
                    i += 1
            elif c == '"':
                in_str = False
        else:
            if c == "/" and nxt == "/":
                in_line = True
                i += 1
            elif c == "/" and nxt == "*":
                in_block = True
                i += 1
            elif c == '"':
                in_str = True
                out.append(c)
            else:
                out.append(c)
        i += 1
    s = "".join(out)
    return re.sub(r",(\s*[}\]])", r"\1", s)      # trailing commas


def load_jsonc(path, default):
    if not os.path.exists(path):
        return default
    raw = open(path, encoding="utf-8").read()
    if not raw.strip():
        return default
    return json.loads(strip_jsonc(raw))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true", help="write the changes")
    ap.add_argument("--file", help="path to keybindings.json (default: autodetect)")
    a = ap.parse_args()

    wanted = load_jsonc(EXAMPLE, [])
    if not wanted:
        print(f"no bindings found in {EXAMPLE}", file=sys.stderr)
        return 1

    # Cross-check against tasks.json before touching anything. Installing a
    # binding whose label does not exist just reintroduces the silent-picker
    # bug at the source, which is the whole thing this is meant to prevent.
    labels = {t["label"] for t in load_jsonc(TASKS, {"tasks": []})["tasks"]}
    bad = [b["args"] for b in wanted if b.get("args") not in labels]
    if bad:
        print("REFUSING: these bindings name tasks that do not exist in tasks.json:")
        for x in bad:
            print(f"    {x!r}")
        print("  Fix .vscode/tasks.json or the example file so they agree.")
        return 1
    print(f"  {len(wanted)} binding(s), all matching a real task label")

    target = a.file
    if not target:
        found = [p for p in (os.path.expanduser(c) for c in CANDIDATES)
                 if os.path.exists(os.path.dirname(p))]
        if not found:
            print("Could not find a VS Code user directory. Pass --file explicitly.",
                  file=sys.stderr)
            return 1
        target = found[0]
        if len(found) > 1:
            print(f"  (found {len(found)} editors; using {target})")
    print(f"  target: {target}")

    existing = load_jsonc(target, [])
    if not isinstance(existing, list):
        print("existing keybindings.json is not a JSON array — not touching it",
              file=sys.stderr)
        return 1

    # Ours = any runTask binding pointing at one of OUR task labels, plus any
    # stale one still pointing at a historical label. That second part is the
    # bit that matters: renaming "sync only  (<glyphs>S)" to "sync" left an
    # orphan behind that silently did nothing.
    ours_now = {b["args"] for b in wanted}
    ours_keys = {b["key"] for b in wanted}

    def is_ours(b):
        if b.get("command") != RUNTASK:
            return False
        args = b.get("args")
        if isinstance(args, str):
            if args in ours_now:
                return True
            # historical labels carried the shortcut inline in brackets
            base = re.sub(r"\s*\(.*\)\s*$", "", args).strip()
            if base in ours_now or args.split("  ")[0].strip() in ours_now:
                return True
            if b.get("key") in ours_keys:
                return True
        return False

    kept = [b for b in existing if not is_ours(b)]
    removed = [b for b in existing if is_ours(b)]

    print()
    for b in removed:
        print(f"  - remove  {b.get('key','?'):<20} {b.get('args')!r}")
    for b in wanted:
        print(f"  + add     {b['key']:<20} {b['args']!r}")
    print(f"  = keeping {len(kept)} unrelated binding(s) untouched")

    if not a.apply:
        print("\n  dry run — nothing written. Re-run with --apply")
        return 0

    if os.path.exists(target):
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = f"{target}.bak-{stamp}"
        shutil.copy2(target, backup)
        print(f"\n  backed up to {backup}")

    parent = os.path.dirname(target)
    if parent:                       # bare filename -> dirname is "", makedirs("") throws
        os.makedirs(parent, exist_ok=True)
    body = json.dumps(kept + wanted, indent=4, ensure_ascii=False)
    header = ("// Managed in part by balance_bot/tools/install_keybindings.py\n"
              "// Entries below that run 'workbench.action.tasks.runTask' for\n"
              "// balance_bot tasks are regenerated by that script; anything\n"
              "// else in this file is left alone.\n")
    with open(target, "w", encoding="utf-8") as f:
        f.write(header + body + "\n")

    json.loads(strip_jsonc(open(target, encoding="utf-8").read()))   # verify
    print(f"  wrote {target}  ({len(kept) + len(wanted)} bindings, valid JSON)")
    print("\n  VS Code picks this up immediately — no restart needed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
