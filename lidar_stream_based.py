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

    print("\nAngle  Value")
    print("----------------")

    samples = []
    last_time = time.time()

    try:
        for pkt in read_packets(ser):

            value = pkt[3]

            samples.append(value)

            # every ~0.1s, assume a revolution chunk
            if time.time() - last_time > 0.1:

                n = len(samples)

                for i, v in enumerate(samples):
                    angle = int(i * 360 / n)
                    print(f"{angle:3d}°   {v:3d}")

                print("---- chunk ----\n")

                samples = []
                last_time = time.time()

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
