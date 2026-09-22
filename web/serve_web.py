#!/usr/bin/env python3
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# This file is part of balance_bot, licensed under the PolyForm
# Noncommercial License 1.0.0. You may use, study, modify, and share
# it for any noncommercial purpose. Commercial use requires a separate
# license from the author -- contact ryan.lush@gmail.com.
# Full license text: see the LICENSE file in the project root, or
# https://polyformproject.org/licenses/noncommercial/1.0.0/

"""
serve_web.py — serve BBotHUD web dashboard on port 8888.
Run from the balance_bot directory:
    python3 web/serve_web.py
Then open http://boneblue-0:8888 in any browser on the network.
"""
import http.server, socketserver, os, sys
from email.utils import formatdate

PORT = 8888
# abspath: os.path.dirname(__file__) is '' when this is run as
# `cd web && python3 serve_web.py`, and os.chdir('') raises FileNotFoundError.
WEB_DIR = os.path.dirname(os.path.abspath(__file__))

# The dashboard that matches current firmware.
#
# v0..v5 all parse D2 telemetry with normPID() — they read
# setpoint/measurement/error/p_term, which the firmware no longer emits, so every
# D2 series silently plots zero. They also contain no SBUS parsing whatsoever,
# so the RC panel does not exist in any of them.
CURRENT = "bbot_dashboard_v6.html"

BANNER = (
    '<div style="position:fixed;top:0;left:0;right:0;z-index:99999;'
    'background:#b3261e;color:#fff;font:13px/1.5 system-ui,sans-serif;'
    'padding:8px 14px;text-align:center">'
    'Outdated dashboard — no RC panel, and D2 telemetry will read zero against '
    'current firmware. '
    f'<a href="/{CURRENT}" style="color:#fff;font-weight:600">Open v6</a>'
    '</div><div style="height:36px"></div>'
)

os.chdir(WEB_DIR)

def _stale_names():
    """Names that get the warning banner injected, so an old URL in someone's
    bookmarks announces itself instead of quietly lying.

    bbot_dashboard.html is deliberately NOT hardcoded in here. It is a symlink,
    and it has since been repointed at v6 — listing it unconditionally meant
    that URL served the *current* dashboard under a red banner claiming it was
    outdated, which is precisely the quiet lie this banner exists to prevent.
    Resolve it instead and flag it only if it really is stale.
    """
    names = {f"bbot_dashboard_v{n}.html" for n in range(6)}
    generic = "bbot_dashboard.html"

    if os.path.islink(generic):
        if os.path.basename(os.readlink(generic)) != CURRENT:
            names.add(generic)
    elif os.path.isfile(generic):
        # A plain copy of unknown vintage. Byte-compare against the current
        # dashboard rather than guessing from the name.
        try:
            with open(generic, "rb") as a, open(CURRENT, "rb") as b:
                if a.read() != b.read():
                    names.add(generic)
        except OSError:
            names.add(generic)
    return names

STALE = _stale_names()

# Never compete with the control loop. (balance_bot also runs SCHED_FIFO now,
# so this is belt and braces.)
try:
    os.nice(19)
except OSError:
    pass


class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # quiet — balance_bot owns the console

    def _serve_ring(self):
        """Last N seconds of the 100 Hz ring, done with byte slicing only --
        no per-line Python work. The old version parsed all ~10 MB line by line
        on this single-core board. Output: the config block in force (segment
        header, or the last '# CHANGE' block before the window), the column row,
        then the rows. ?sec=0 returns everything."""
        from urllib.parse import urlparse, parse_qs
        segs = [p for p in ("/dev/shm/bbot_ring_0.csv", "/dev/shm/bbot_ring_1.csv")
                if os.path.isfile(p)]
        if not segs:
            self._text(404, "no 100 Hz ring in /dev/shm - is balance_bot running a build with ring_tick?")
            return
        segs.sort(key=os.path.getmtime)                 # oldest first
        try:
            sec = max(0.0, float(parse_qs(urlparse(self.path).query).get("sec", ["60"])[0]))
        except ValueError:
            sec = 60.0
        data = [open(p, "rb").read() for p in segs]     # 10 MB max, from RAM
        # drop a partially written last line of the newest segment
        cut = data[-1].rfind(b"\n")
        data[-1] = data[-1][:cut + 1] if cut >= 0 else b""

        def split_header(buf):
            i = buf.find(b"\nt,")
            if i < 0:
                return b"", buf
            j = buf.find(b"\n", i + 1)
            return buf[:j + 1], buf[j + 1:]

        hdr, _ = split_header(data[0])
        rows = b"".join(split_header(d)[1] for d in data)
        if sec > 0:
            want = int(sec * 100 * 260)                 # ~260 bytes/row, generous
            if len(rows) > want:
                start = rows.find(b"\n", len(rows) - want) + 1
                before, rows = rows[:start], rows[start:]
                k = before.rfind(b"# CHANGE")            # config changed before window?
                if k >= 0:
                    blk = before[k:]
                    cfg = b"".join(l + b"\n" for l in blk.split(b"\n") if l.startswith(b"#"))
                    cols = hdr[hdr.find(b"\nt,") + 1:]
                    hdr = cfg + cols
        body = hdr + rows
        self.send_response(200)
        self.send_header("Content-Type", "text/csv")
        self.send_header("Content-Disposition", 'attachment; filename="bbot_ring.csv"')
        # The dashboard may be opened from a file on the Mac, not served from here.
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _text(self, code, msg):
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write((msg + "\n").encode())

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", f"/{CURRENT}")
            self.end_headers()
            return

        name = self.path.lstrip("/").split("?")[0]

        # Always-on 100 Hz ring (robot.c ring_tick): two segments in /dev/shm.
        # Joined oldest-first into one CSV. ?sec=N keeps only the last N seconds.
        if name == "bbot_ring.csv":
            self._serve_ring()
            return
        if name in STALE and os.path.isfile(name):
            with open(name, "rb") as fh:
                body = fh.read()
            lower = body.lower()
            i = lower.find(b"<body")
            if i != -1:
                i = lower.find(b">", i) + 1
                body = body[:i] + BANNER.encode() + body[i:]
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        # Serve the dashboard itself uncached, with its own mtime stamped into
        # the page. Without this a browser keeps showing a page from before the
        # last rsync and there is no way to tell that from the firmware not
        # having the feature -- which has now cost several debugging rounds.
        if name.endswith(".html") and os.path.isfile(name):
            with open(name, "rb") as fh:
                body = fh.read()
            import time
            stamp = time.strftime("%H:%M:%S", time.localtime(os.path.getmtime(name)))
            body = body.replace(b"__BUILD__", stamp.encode())
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        return super().do_GET()

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"BBotHUD web dashboard: http://boneblue-0:{PORT}/  ->  {CURRENT}", flush=True)
    httpd.serve_forever()
