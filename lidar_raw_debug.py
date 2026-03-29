#!/usr/bin/env python3

import serial
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200

HEADER = b'\x5a\xa5'

ser = serial.Serial(PORT, BAUD, timeout=0.1)

print("Starting lidar...")
ser.write(b"startlds$")

buf = b''

while True:
    buf += ser.read(256)

    while True:
        idx = buf.find(HEADER)
        if idx == -1:
            buf = buf[-8:]
            break

        # print next 32 bytes from header
        if len(buf) < idx + 32:
            break

        pkt = buf[idx:idx+32]

        print(pkt.hex())

        buf = buf[idx+2:]
