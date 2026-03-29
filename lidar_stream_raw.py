#!/usr/bin/env python3

import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200

HEADER = b'\x5a\xa5'

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0)
    time.sleep(0.2)

    print("Starting lidar...")
    ser.write(b"startlds$")

    buf = b''

    try:
        while True:
            data = ser.read(1024)
            if data:
                buf += data

            # hard sync loop
            while True:
                idx = buf.find(HEADER)
                if idx == -1:
                    # keep only tail so buffer doesn't explode
                    if len(buf) > 4:
                        buf = buf[-4:]
                    break

                # ensure full 4-byte frame
                if len(buf) < idx + 4:
                    break

                pkt = buf[idx:idx+4]
                buf = buf[idx+4:]

                # print raw, ALWAYS
                print(pkt.hex())

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
