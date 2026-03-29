#!/usr/bin/env python3

import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200

HEADER0 = 0x5A
HEADER1 = 0xA5

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0)
    time.sleep(0.2)

    print("Starting lidar...")
    ser.write(b"startlds$")

    state = 0

    try:
        while True:
            b = ser.read(1)
            if not b:
                continue

            b = b[0]

            # state machine (no buffer, no desync possible)
            if state == 0:
                if b == HEADER0:
                    state = 1

            elif state == 1:
                if b == HEADER1:
                    state = 2
                else:
                    state = 0

            elif state == 2:
                byte2 = b
                state = 3

            elif state == 3:
                byte3 = b

                # ALWAYS print packet
                print(f"5aa5{byte2:02x}{byte3:02x}")

                state = 0

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
