#!/usr/bin/env python3
"""
LDS-007 / LDS-006 Ecovacs LiDAR reader
Protocol: 115200 8N1, packets start with 0xFA
90 packets/revolution, 4 samples/packet = 360 samples/rev

Usage:
    pip install pyserial
    python3 lds007.py /dev/tty.usbserial-XXXX
"""

import serial
import struct
import sys
import time
import math

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-A10L0VXU"
BAUD = 115200
PACKET_LEN = 22     # bytes per packet
START_BYTE = 0xFA
START_INDEX = 0xA0  # first packet index
END_INDEX   = 0xF9  # last packet index (90 packets total)

def calc_checksum(data):
    """
    Neato/Ecovacs LDS checksum:
    Sum 10 uint16 little-endian words from bytes 0..19, then fold carry.
    """
    chk = 0
    for i in range(0, 20, 2):
        chk = (chk << 1) + struct.unpack_from('<H', data, i)[0]
    chk = (chk + (chk >> 15)) & 0x7FFF
    return chk

def parse_packet(data):
    """
    Parse one 22-byte LDS packet.

    Packet layout:
      [0]      0xFA  start byte
      [1]      index 0xA0..0xF9
      [2..3]   speed (RPM * 64), little-endian uint16
      [4..17]  4 x 4-byte samples
      [18..19] checksum, little-endian uint16
      [20..21] (some variants pad to 22; treat as reserved)

    Each 4-byte sample:
      [0..1]  distance mm (14 bits), flags in upper 2 bits of byte[1]
      [2..3]  signal strength / quality
    """
    if len(data) < 22 or data[0] != START_BYTE:
        return None

    index = data[1]
    if not (START_INDEX <= index <= END_INDEX):
        return None

    speed_raw = struct.unpack_from('<H', data, 2)[0]
    rpm = speed_raw / 64.0

    # Checksum covers bytes 0..19
    expected_chk = calc_checksum(data[:20])
    actual_chk   = struct.unpack_from('<H', data, 18)[0]
    chk_ok = (expected_chk == actual_chk)

    samples = []
    base_angle = (index - START_INDEX) * 4  # each packet covers 4 degrees
    for i in range(4):
        offset = 4 + i * 4
        b0, b1, b2, b3 = data[offset], data[offset+1], data[offset+2], data[offset+3]

        flag_no_detect = bool(b1 & 0x80)  # bit 15: no object detected
        flag_warning   = bool(b1 & 0x40)  # bit 14: signal warning
        distance_mm    = ((b1 & 0x3F) << 8) | b0
        quality        = (b3 << 8) | b2

        angle_deg = base_angle + i
        samples.append({
            "angle":       angle_deg,
            "distance_mm": distance_mm if not flag_no_detect else None,
            "quality":     quality,
            "warning":     flag_warning,
            "no_detect":   flag_no_detect,
        })

    return {
        "index":   index,
        "rpm":     rpm,
        "chk_ok":  chk_ok,
        "samples": samples,
    }

def find_packet(ser):
    """Scan the byte stream until we find a valid 0xFA start + valid index."""
    while True:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != START_BYTE:
            continue
        # Peek at the index byte
        idx = ser.read(1)
        if not idx:
            continue
        if START_INDEX <= idx[0] <= END_INDEX:
            # Read the rest of the packet
            rest = ser.read(PACKET_LEN - 2)
            if len(rest) == PACKET_LEN - 2:
                return bytes([START_BYTE, idx[0]]) + rest

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.1)

    # Send the wake/start command (0x24 = '$')
    # The LDS-006/007 responds with 0x21 ('!') and begins streaming.
    print("Sending start command 0x24 ('$')...")
    ser.write(bytes([0x24]))
    time.sleep(0.1)

    resp = ser.read(ser.in_waiting or 1)
    if resp:
        print(f"  Got response: {resp.hex()}  ({repr(resp)})")
    else:
        print("  No immediate response — unit may already be running or motor not spinning yet.")

    print("\nListening for packets... (Ctrl-C to stop)\n")
    print(f"{'Angle':>6}  {'Dist(mm)':>9}  {'Quality':>7}  {'RPM':>6}  {'ChkOK':>5}  Flags")
    print("-" * 55)

    scan = {}  # angle -> distance for one full revolution

    try:
        while True:
            raw = find_packet(ser)
            if raw is None:
                continue

            pkt = parse_packet(raw)
            if pkt is None:
                continue

            if not pkt["chk_ok"]:
                sys.stderr.write(f"[chk fail idx={pkt['index']:#x}]\n")
                continue

            for s in pkt["samples"]:
                scan[s["angle"]] = s["distance_mm"]

                flags = []
                if s["no_detect"]: flags.append("NO_OBJ")
                if s["warning"]:   flags.append("WARN")
                flag_str = ",".join(flags) if flags else "OK"

                dist_str = f"{s['distance_mm']:>9}" if s["distance_mm"] is not None else "     None"
                print(f"{s['angle']:>6}°  {dist_str}  {s['quality']:>7}  {pkt['rpm']:>6.1f}  {'Y' if pkt['chk_ok'] else 'N':>5}  {flag_str}")

            # Print a summary line each full revolution
            if pkt["index"] == END_INDEX:
                valid = sum(1 for v in scan.values() if v is not None)
                avg_dist = (sum(v for v in scan.values() if v is not None) / valid) if valid else 0
                print(f"\n--- Revolution complete: {valid}/360 valid readings, avg dist {avg_dist:.0f} mm ---\n")
                scan.clear()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()