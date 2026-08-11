#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
serve_web.py — serve BBotHUD web dashboard on port 8888.
Run from the balance_bot directory:
    python3 web/serve_web.py
Then open http://boneblue-0:8888 in any browser on the network.
"""
import http.server, socketserver, os, sys

PORT = 8888
WEB_DIR = os.path.join(os.path.dirname(__file__))

# The dashboard that matches current firmware.
#
# bbot_dashboard.html is a symlink to v4, and v0..v5 all parse D2 telemetry with
# normPID() — they read setpoint/measurement/error/p_term, which the firmware no
# longer emits, so every D2 series silently plots zero. Serving by explicit name
# rather than relying on the symlink, which cannot be repointed here.
CURRENT = "bbot_dashboard_v6.html"

# Anything in this list is served with a warning banner injected, so an old URL
# in someone's bookmarks announces itself instead of quietly lying.
STALE = {f"bbot_dashboard_v{n}.html" for n in range(6)} | {"bbot_dashboard.html"}

BANNER = (
    '<div style="position:fixed;top:0;left:0;right:0;z-index:99999;'
    'background:#b3261e;color:#fff;font:13px/1.5 system-ui,sans-serif;'
    'padding:8px 14px;text-align:center">'
    'Outdated dashboard — D2 telemetry will read zero against current firmware. '
    f'<a href="/{CURRENT}" style="color:#fff;font-weight:600">Open v6</a>'
    '</div><div style="height:36px"></div>'
)

os.chdir(WEB_DIR)


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

        return super().do_GET()


with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"BBotHUD web dashboard: http://boneblue-0:{PORT}/  ->  {CURRENT}", flush=True)
    httpd.serve_forever()
