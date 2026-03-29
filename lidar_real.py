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

            # need at least header + len byte
            if len(buf) < idx + 3:
                break

            length = buf[idx+2]

            total_len = 3 + length  # header + len + payload

            if len(buf) < idx + total_len:
                break

            pkt = buf[idx:idx+total_len]
            buf = buf[idx+total_len:]

            yield pkt

def parse_packet(pkt):
    # pkt = [5A A5 LEN ...payload...]

    length = pkt[2]
    payload = pkt[3:]

    samples = []

    # assume payload = repeating (angle, dist)
    # this is heuristic but works for your stream

    for i in range(0, len(payload)-1, 2):
        angle = payload[i]
        dist  = payload[i+1]

        samples.append((angle, dist))

    return samples

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.2)

    start_lidar(ser)

    print("\nAngle  Dist")
    print("----------------")

    scan = [None]*360

    try:
        for pkt in read_stream(ser):

            samples = parse_packet(pkt)

            for angle_raw, dist in samples:

                angle = int(angle_raw * 360 / 256)

                scan[angle] = dist

                print(f"{angle:3d}°   {dist:3d}")

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
