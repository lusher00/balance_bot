#!/usr/bin/env python3
"""
serve_web.py — serve BBotHUD web dashboard on port 8888.
Run from the balance_bot directory:
    python3 web/serve_web.py
Then open http://boneblue-0:8888 in any browser on the network.
"""
import http.server, socketserver, os, sys

PORT = 8888
WEB_DIR = os.path.join(os.path.dirname(__file__))

os.chdir(WEB_DIR)

class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # quiet — balance_bot owns the console

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"BBotHUD web dashboard: http://boneblue-0:{PORT}/bbot_dashboard.html", flush=True)
    httpd.serve_forever()
