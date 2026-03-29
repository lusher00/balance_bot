#!/usr/bin/env python3

import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"

ser = serial.Serial(PORT, 115200, timeout=0)

time.sleep(0.2)
ser.write(b"startlds$")

state = 0

while True:
    b = ser.read(1)
    if not b:
        continue

    b = b[0]

    if state == 0:
        if b == 0x5A:
            state = 1

    elif state == 1:
        if b == 0xA5:
            state = 2
        else:
            state = 0

    elif state == 2:
        b2 = b
        state = 3

    elif state == 3:
        b3 = b

        # ALWAYS print, no filtering
        print(f"{b2:02x} {b3:02x}")

        state = 0
