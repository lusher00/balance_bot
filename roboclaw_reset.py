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
roboclaw_reset.py — reset RoboClaw via WriteNVM (cmd 94) using stdlib only.
No pyserial dependency — uses termios directly.
Always exits 0 so a failure never blocks the service from starting.
"""

import os, sys, struct, time, termios, tty

PORT = '/dev/ttyO1'
BAUD = 460800
ADDR = 0x80

BAUD_MAP = {
    460800: termios.B460800,
    115200: termios.B115200,
     38400: termios.B38400,
      9600: termios.B9600,
}

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    os.set_blocking(fd, True)
    attrs = termios.tcgetattr(fd)
    attrs[4] = BAUD_MAP[baud]   # ispeed
    attrs[5] = BAUD_MAP[baud]   # ospeed
    attrs[0] = 0                # iflag — raw
    attrs[1] = 0                # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0                # lflag
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

def send_acked(fd, payload, timeout=1.0):
    crc = crc16(payload)
    os.write(fd, payload + struct.pack('>H', crc))
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            b = os.read(fd, 1)
            if b and b[0] == 0xFF:
                return True
        except BlockingIOError:
            time.sleep(0.01)
    return False

try:
    fd = open_port(PORT, BAUD)
    time.sleep(0.1)

    payload = struct.pack('>BBI', ADDR, 94, 0xE22EAB7A)
    ok = send_acked(fd, payload, timeout=1.0)
    os.close(fd)

    if ok:
        print("roboclaw_reset: WriteNVM OK — waiting 2s for unit to come up")
    else:
        print("roboclaw_reset: no ACK — unit may already be reset, continuing")

    time.sleep(2.0)

except Exception as e:
    print(f"roboclaw_reset: error — {e}, continuing anyway")

sys.exit(0)
