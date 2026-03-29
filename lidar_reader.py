#!/usr/bin/env python3

import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200

HEADER = b'\x5a\xa5'

def open_serial():
    print(f"Opening {PORT} @ {BAUD}")
    return serial.Serial(PORT, BAUD, timeout=1)

def start_lidar(ser):
    print("Sending start command '$'")
    ser.write(b'\x24')
    time.sleep(0.1)

    resp = ser.read(ser.in_waiting or 1)
    if resp:
        print(f"Response: {resp.hex()}")

def read_stream(ser):
    buf = b''

    while True:
        buf += ser.read(512)

        while True:
            idx = buf.find(HEADER)
            if idx == -1:
                # prevent buffer from growing forever
                if len(buf) > 2048:
                    buf = buf[-32:]
                break

            if len(buf) < idx + 4:
                break

            pkt = buf[idx:idx+4]
            buf = buf[idx+4:]

            yield pkt

def decode_packet(pkt):
    # packet looks like: 5a a5 XX YY
    if len(pkt) < 4:
        return None

    b0, b1, b2, b3 = pkt

    # crude decoding (based on observed pattern)
    # distance-ish value is in last byte
    val = b3

    return val

def main():
    ser = open_serial()
    time.sleep(0.1)

    start_lidar(ser)

    print("\nStreaming...\n")

    try:
        for pkt in read_stream(ser):
            val = decode_packet(pkt)

            # print raw + interpreted
            print(f"{pkt.hex()}  ->  {val}")

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
