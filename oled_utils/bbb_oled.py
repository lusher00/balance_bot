#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
"""
bbb_oled.py — SSD1306 status display for the BeagleBone.

Shows IP, the state of a systemd unit, battery voltage, the clock, and a
heartbeat dot that blinks while the watched service is running.

Design notes, since this runs unattended as a service:

  * Nothing is allowed to fail silently. The original swallowed every
    exception with a bare `except Exception: pass`, so an unreadable font, a
    wrong I2C address and a dead bus all looked identical — a blank screen and
    an empty journal. Failures are logged, once per distinct cause, and the
    loop keeps going.
  * It clears the panel on SIGTERM. Without that, `systemctl stop` leaves the
    last frame burned on screen indefinitely and it looks like it's still up.
  * The loop is scheduled against a monotonic deadline, so a slow I2C write
    doesn't make the clock drift.
  * `systemctl is-active` is a fork+exec. Doing that every second on an AM335x
    is wasteful, so the status is cached and refreshed on its own slower
    interval while the display keeps redrawing.

  ./bbb_oled.py --i2c-port 1 --service balance_bot
  ./bbb_oled.py --once            # one frame, then exit — for testing
  ./bbb_oled.py --list-fonts      # what's actually installed
"""

import argparse
import json
import logging
import os
import signal
import socket
import subprocess
import sys
import time

LOG = logging.getLogger("bbb_oled")

DEFAULT_I2C_ADDR = 0x3C          # some panels are 0x3D
BATT_STATUS_PATH = "/run/batt_status.json"
FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
)
BATT_STALE_SEC = 30.0            # older than this and we show it as unknown


class Once:
    """Log a given message only when it changes.

    A display loop repeats its errors every second. Without this the journal
    fills with thousands of identical lines and the real first failure scrolls
    away.
    """

    def __init__(self):
        self._last = {}

    def log(self, key, level, msg, *args):
        text = msg % args if args else msg
        if self._last.get(key) != text:
            LOG.log(level, text)
            self._last[key] = text

    def clear(self, key):
        self._last.pop(key, None)


ONCE = Once()


# ── data sources ────────────────────────────────────────────────────

def get_ip():
    """Local address of the default route. No packets are sent."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(0.5)
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError as e:
        ONCE.log("ip", logging.INFO, "no IP (%s)", e)
        return "no ip"


def get_service_status(service_name):
    try:
        out = subprocess.run(
            ["systemctl", "is-active", service_name],
            capture_output=True, text=True, timeout=2,
        )
        status = out.stdout.strip()
        return status or "unknown"
    except (OSError, subprocess.SubprocessError) as e:
        ONCE.log("svc", logging.WARNING, "systemctl is-active failed: %s", e)
        return "unknown"


def get_battery_voltage(path):
    """Battery volts from the status file, or None.

    Returns None when the file is missing, malformed, or stale — a number that
    stopped updating an hour ago is worse than no number.
    """
    try:
        st = os.stat(path)
        if time.time() - st.st_mtime > BATT_STALE_SEC:
            ONCE.log("batt", logging.INFO, "%s is stale (%.0fs old)",
                     path, time.time() - st.st_mtime)
            return None
        with open(path) as fh:
            data = json.load(fh)
    except FileNotFoundError:
        ONCE.log("batt", logging.INFO, "%s not present", path)
        return None
    except (OSError, ValueError) as e:
        ONCE.log("batt", logging.WARNING, "cannot read %s: %s", path, e)
        return None

    for key in ("voltage", "voltage_v", "v"):
        if key in data:
            try:
                ONCE.clear("batt")
                return float(data[key])
            except (TypeError, ValueError):
                break
    ONCE.log("batt", logging.WARNING, "no voltage key in %s (have: %s)",
             path, ", ".join(sorted(data)) if isinstance(data, dict) else type(data).__name__)
    return None


# ── display ─────────────────────────────────────────────────────────

def load_font(explicit, size):
    from PIL import ImageFont
    candidates = [explicit] if explicit else list(FONT_CANDIDATES)
    for path in candidates:
        if path and os.path.exists(path):
            try:
                return ImageFont.truetype(path, size), path
            except OSError as e:
                LOG.warning("font %s unusable: %s", path, e)
    LOG.warning("no TrueType font found (tried: %s) — falling back to PIL's "
                "built-in bitmap font, which ignores --font-size",
                ", ".join(c for c in candidates if c))
    return ImageFont.load_default(), "<default>"


class Display:
    """Owns the panel. Reconnects on I2C errors rather than dying."""

    def __init__(self, port, addr, width, height):
        self.port, self.addr = port, addr
        self.width, self.height = width, height
        self.device = None

    def connect(self):
        from luma.core.interface.serial import i2c
        from luma.oled.device import ssd1306
        serial = i2c(port=self.port, address=self.addr)
        self.device = ssd1306(serial, width=self.width, height=self.height)
        # luma clears at exit by default; we do it explicitly in shutdown()
        # so the ordering is ours and works under SIGTERM too.
        self.device.persist = True
        ONCE.clear("connect")
        LOG.info("connected to SSD1306 at i2c-%d 0x%02X (%dx%d)",
                 self.port, self.addr, self.width, self.height)

    def ensure(self):
        if self.device is not None:
            return True
        try:
            self.connect()
            return True
        except Exception as e:      # luma raises a wide variety here
            ONCE.log("connect", logging.ERROR,
                     "cannot open SSD1306 at i2c-%d 0x%02X: %s "
                     "(check the bus exists and the address is right: "
                     "i2cdetect -y -r %d)", self.port, self.addr, e, self.port)
            return False

    def drop(self, why):
        ONCE.log("draw", logging.WARNING, "display error, reconnecting: %s", why)
        self.device = None

    def shutdown(self):
        """Blank the panel so a stopped service doesn't look like a live one."""
        if self.device is None:
            return
        try:
            self.device.clear()
            self.device.hide()
        except Exception as e:
            LOG.warning("could not clear display on exit: %s", e)
        finally:
            self.device = None


def render(display, font, rows, beat, show_beat):
    from luma.core.render import canvas

    line_h = 12 if display.height >= 64 else 10
    with canvas(display.device) as draw:
        y = 0
        for label, value in rows:
            if y + line_h > display.height:
                break
            draw.text((3, y), f"{label}{value}", font=font, fill="white")
            y += line_h

        # Heartbeat, bottom-left, only if there is room below the text.
        top = display.height - 14
        if top >= y:
            box = (3, top, 13, top + 10)
            if show_beat:
                draw.ellipse(box, outline="white", fill="white" if beat else "black")
            else:
                draw.ellipse(box, outline="white", fill="black")
                draw.line((box[0], box[1], box[2], box[3]), fill="white")
                draw.line((box[0], box[3], box[2], box[1]), fill="white")


# ── main ────────────────────────────────────────────────────────────

def hex_or_int(text):
    return int(text, 0)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--service", default="balance_bot",
                   help="systemd unit to monitor (default: balance_bot)")
    p.add_argument("--i2c-port", type=int, default=1,
                   help="I2C bus number (default: 1)")
    p.add_argument("--i2c-addr", type=hex_or_int, default=DEFAULT_I2C_ADDR,
                   help="panel address, e.g. 0x3C or 0x3D (default: 0x3C)")
    p.add_argument("--width", type=int, default=128, help="panel width (default: 128)")
    p.add_argument("--height", type=int, default=64,
                   help="panel height, 64 or 32 (default: 64)")
    p.add_argument("--refresh", type=float, default=1.0,
                   help="redraw interval, seconds (default: 1.0)")
    p.add_argument("--status-interval", type=float, default=2.0,
                   help="how often to shell out to systemctl (default: 2.0)")
    p.add_argument("--batt-path", default=BATT_STATUS_PATH,
                   help=f"battery status JSON (default: {BATT_STATUS_PATH})")
    p.add_argument("--font", default=None, help="TrueType font path")
    p.add_argument("--font-size", type=int, default=9, help="font size (default: 9)")
    p.add_argument("--retry", type=float, default=2.0,
                   help="reconnect delay after an I2C error (default: 2.0)")
    p.add_argument("--once", action="store_true",
                   help="draw a single frame and exit (for testing)")
    p.add_argument("--list-fonts", action="store_true",
                   help="print the fonts this script would try, then exit")
    p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s", stream=sys.stderr)

    if args.list_fonts:
        for path in FONT_CANDIDATES:
            print(f"{'OK     ' if os.path.exists(path) else 'missing'}  {path}")
        return 0

    try:
        import luma.oled  # noqa: F401
        from PIL import ImageFont  # noqa: F401
    except ImportError as e:
        LOG.error("missing dependency: %s", e)
        LOG.error("install with: sudo pip3 install --break-system-packages "
                  "luma.oled pillow")
        return 1

    font, font_path = load_font(args.font, args.font_size)
    LOG.debug("using font %s", font_path)

    display = Display(args.i2c_port, args.i2c_addr, args.width, args.height)

    stopping = False

    def on_signal(signum, _frame):
        nonlocal stopping
        stopping = True
        LOG.info("caught %s, clearing display", signal.Signals(signum).name)

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    beat = False
    svc = "unknown"
    next_status = 0.0
    deadline = time.monotonic()

    try:
        while not stopping:
            now = time.monotonic()
            if now >= next_status:
                svc = get_service_status(args.service)
                next_status = now + max(0.5, args.status_interval)

            if not display.ensure():
                if args.once:
                    return 1
                time.sleep(args.retry)
                deadline = time.monotonic()
                continue

            batt = get_battery_voltage(args.batt_path)
            rows = [
                ("IP:   ", get_ip()),
                ("BBOT: ", svc),
                ("BATT: ", f"{batt:.1f}V" if batt is not None else "------"),
                ("TIME: ", time.strftime("%H:%M:%S")),
            ]

            try:
                render(display, font, rows, beat, svc == "active")
            except Exception as e:
                display.drop(e)
                if args.once:
                    return 1
                time.sleep(args.retry)
                deadline = time.monotonic()
                continue

            ONCE.clear("draw")
            beat = not beat
            if args.once:
                return 0

            # Schedule against a fixed deadline so slow writes don't drift.
            deadline += args.refresh
            sleep_for = deadline - time.monotonic()
            if sleep_for < 0:
                deadline = time.monotonic()
                sleep_for = 0
            time.sleep(sleep_for)
    finally:
        display.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
