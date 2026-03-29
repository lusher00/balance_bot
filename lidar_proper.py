#!/usr/bin/env python3

import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200

HEADER = b'\x5a\xa5'

def start_lidar(ser):
    print("Starting lidar...")
    ser.write(b"startlds$")
    time.sleep(1)

def read_stream(ser):
    buf = b''

    while True:
        buf += ser.read(512)

        while True:
            idx = buf.find(HEADER)
            if idx == -1:
                if len(buf) > 4096:
                    buf = buf[-32:]
                break

            if len(buf) < idx + 4:
                break

            pkt = buf[idx:idx+4]
            buf = buf[idx+4:]

            yield pkt

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.2)

    start_lidar(ser)

    print("\nAngle  Dist")
    print("----------------")

    # full scan buffer
    scan = [None] * 360

    last_print = time.time()

    try:
        for pkt in read_stream(ser):

            # decode packet
            angle = pkt[2]        # index / angle bucket
            dist  = pkt[3]        # raw distance-ish

            # map to 0–359 (scale index if needed)
            # your index is ~0–255 → stretch to 360
            angle_deg = int(angle * 360 / 256)

            scan[angle_deg] = dist

            print(f"{angle_deg:3d}°   {dist:3d}")

            # detect revolution (wraparound)
            if angle_deg < 5 and any(scan[300:]):
                valid = sum(1 for v in scan if v is not None)
                avg = sum(v for v in scan if v is not None) / valid if valid else 0

                print(f"\n--- REV COMPLETE: {valid}/360 pts | avg={avg:.1f} ---\n")

                scan = [None] * 360

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
