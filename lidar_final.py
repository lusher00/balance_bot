#!/usr/bin/env python3

import serial
import sys
import time
import struct

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
                buf = buf[-32:]
                break

            # need minimum header + fields
            if len(buf) < idx + 10:
                break

            # packet structure:
            # 5A A5 CT LSN FSA(2) LSA(2) CS(2)
            ct  = buf[idx+2]
            lsn = buf[idx+3]

            pkt_len = 10 + lsn * 2

            if len(buf) < idx + pkt_len:
                break

            pkt = buf[idx:idx+pkt_len]
            buf = buf[idx+pkt_len:]

            yield pkt

def parse_packet(pkt):
    lsn = pkt[3]

    fsa = struct.unpack_from('<H', pkt, 4)[0]
    lsa = struct.unpack_from('<H', pkt, 6)[0]

    start_angle = (fsa >> 1) / 64.0
    end_angle   = (lsa >> 1) / 64.0

    if end_angle < start_angle:
        end_angle += 360

    samples = []

    for i in range(lsn):
        offset = 10 + i*2
        dist_raw = struct.unpack_from('<H', pkt, offset)[0]

        dist = dist_raw / 4.0  # typical scaling

        angle = start_angle + (end_angle - start_angle) * i / max(lsn-1,1)
        angle = angle % 360

        samples.append((angle, dist))

    return samples

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.2)

    start_lidar(ser)

    print("\nAngle   Dist(mm)")
    print("-----------------------")

    try:
        for pkt in read_stream(ser):
            samples = parse_packet(pkt)

            for angle, dist in samples:
                print(f"{angle:6.1f}°   {dist:6.1f}")

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
