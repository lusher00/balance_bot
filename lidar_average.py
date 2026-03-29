#!/usr/bin/env python3

import serial
import sys
import time
from collections import defaultdict

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200

HEADER = b'\x5a\xa5'

def start_lidar(ser):
    ser.write(b"startlds$")
    time.sleep(1)

def read_packets(ser):
    buf = b''
    while True:
        buf += ser.read(256)

        while True:
            idx = buf.find(HEADER)
            if idx == -1:
                buf = buf[-4:]
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

    samples = defaultdict(list)
    last = time.time()

    try:
        for pkt in read_packets(ser):

            value = pkt[3]
            angle_raw = pkt[2]

            angle = int(angle_raw * 360 / 256)

            samples[angle].append(value)

            # every ~0.1 sec = one revolution chunk
            if time.time() - last > 0.1:

                print("\nAngle  AvgDist")
                print("----------------")

                for a in sorted(samples.keys()):
                    vals = samples[a]
                    avg = sum(vals) / len(vals)

                    print(f"{a:3d}°   {avg:6.1f}")

                samples.clear()
                last = time.time()

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
