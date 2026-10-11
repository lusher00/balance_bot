#!/usr/bin/env python3
"""Merge [section] key = value overrides into an INI-style robot.conf in place.

Everything not named in the overrides is left byte-for-byte as it was: comments,
order, unknown keys. A key missing from its section is appended to that
section; a missing section is appended at the end. A timestamped backup is
written next to the target first.

    conf_patch.py <target robot.conf> <overrides file>
"""
import re, shutil, sys, time

SEC = re.compile(r'^\s*\[([^\]]+)\]\s*$')
KV = re.compile(r'^(\s*)([A-Za-z0-9_]+)(\s*=\s*)(.*?)(\s*(#.*)?)$')

def read_overrides(path):
    out = {}  # section -> [(key, value)]
    sec = None
    for line in open(path):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        m = SEC.match(s)
        if m:
            sec = m.group(1).strip().lower(); out.setdefault(sec, []); continue
        if '=' not in s:
            continue
        k, v = s.split('=', 1)
        if sec is None:
            sys.exit('override %r is outside any [section]' % s)
        out[sec].append((k.strip(), v.strip()))
    return out

def same_value(a, b):
    """0.1400 and 0.14 are the same setting; only a real change is written."""
    try:
        return float(a) == float(b)
    except ValueError:
        return a.strip() == b.strip()

def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    target, ovpath = sys.argv[1], sys.argv[2]
    ov = read_overrides(ovpath)
    lines = open(target).read().split('\n')
    backup = '%s.bak.%s' % (target, time.strftime('%Y%m%d-%H%M%S'))
    shutil.copy2(target, backup)

    # index: section -> (start line, end line exclusive)
    spans, cur, start = {}, None, 0
    for i, line in enumerate(lines):
        m = SEC.match(line)
        if m:
            if cur is not None:
                spans[cur] = (start, i)
            cur, start = m.group(1).strip().lower(), i + 1
    if cur is not None:
        spans[cur] = (start, len(lines))

    changed = []
    for sec, kvs in ov.items():
        if sec not in spans:
            # new section at the end
            if lines and lines[-1].strip():
                lines.append('')
            lines.append('[%s]' % sec)
            for k, v in kvs:
                lines.append('%s = %s' % (k, v)); changed.append((sec, k, None, v))
            continue
        s, e = spans[sec]
        for k, v in kvs:
            hit = None
            for i in range(s, e):
                m = KV.match(lines[i])
                if m and m.group(2).lower() == k.lower():
                    hit = i; break
            if hit is None:
                # append just before the section's trailing blank lines
                ins = e
                while ins > s and not lines[ins - 1].strip():
                    ins -= 1
                lines.insert(ins, '%s = %s' % (k, v))
                # shift later spans
                for sk, (ss, se) in spans.items():
                    if ss > ins: spans[sk] = (ss + 1, se + 1)
                    elif se >= ins and sk == sec: spans[sk] = (ss, se + 1)
                e += 1
                changed.append((sec, k, None, v))
            else:
                m = KV.match(lines[hit]); old = m.group(4)
                if not same_value(old, v):
                    lines[hit] = '%s%s%s%s%s' % (m.group(1), m.group(2), m.group(3), v, m.group(5) or '')
                    changed.append((sec, k, old, v))
    open(target, 'w').write('\n'.join(lines))
    print('backup: %s' % backup)
    if not changed:
        print('no changes (every override already matched)')
    for sec, k, old, v in changed:
        print('  [%s] %s: %s -> %s' % (sec, k, old if old is not None else '(absent)', v))

if __name__ == '__main__':
    main()
