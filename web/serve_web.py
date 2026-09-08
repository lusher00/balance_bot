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

class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # quiet — balance_bot owns the console

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", f"/{CURRENT}")
            self.end_headers()
            return

        name = self.path.lstrip("/").split("?")[0]

        # The loop-rate capture lands in /tmp on the bot, which is nowhere the
        # operator can reach from a browser. Serve it from here so the dashboard
        # can just hand over the file -- no scp, no second tool, no wondering
        # where it went.
        if name == "bbot.csv":
            path = "/tmp/bbot.csv"
            if not os.path.isfile(path):
                self.send_response(404)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(b"no capture yet - press CAPTURE AT LOOP RATE first\n")
                return
            with open(path, "rb") as fh:
                body = fh.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", 'attachment; filename="bbot.csv"')
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
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
