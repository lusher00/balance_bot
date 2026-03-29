#!/usr/bin/env python3

import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200

HEADER = b'\x5a\xa5'

def open_serial():
    print(f"Opening {PORT} @ {BAUD}")
    return serial.Serial(PORT, BAUD, timeout=0.1)

def try_start_sequences(ser):
    print("\n=== Trying start sequences ===")

    # 1. Known working command for LDS clones
    print("Trying: startlds$")
    ser.write(b"startlds$")
    time.sleep(1)

    # 2. Fallback: classic $
    print("Trying: $")
    ser.write(b'\x24')
    time.sleep(1)

    # 3. PWM kick (some units need this)
    print("Trying: PWM kick")
    for pwm in [0x60, 0x70, 0x80, 0x90]:
        print(f"  PWM {pwm}")
        ser.write(bytes([0x21, pwm, 0x21]))
        time.sleep(0.5)

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
    ser = open_serial()
    time.sleep(0.2)

    try_start_sequences(ser)

    print("\n=== Streaming ===\n")

    last = time.time()
    count = 0

    try:
        for pkt in read_stream(ser):
            count += 1

            # crude "distance-ish" extraction
            val = pkt[3]

            print(f"{pkt.hex()}  -> {val}")

            # periodic status
            if time.time() - last > 2:
                print(f"\n[rate: {count/2:.0f} pkt/sec]\n")
                count = 0
                last = time.time()

    except KeyboardInterrupt:
        print("\nStopped")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
