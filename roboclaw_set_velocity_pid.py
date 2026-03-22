#!/usr/bin/env python3
"""
Set RoboClaw M1/M2 velocity PID gains and save to flash.

Commands:
  28 = SetM1VelocityPID
  29 = SetM2VelocityPID
  94 = WriteNVM (save to flash)

Packet format for velocity PID (all uint32, big-endian):
  [addr, cmd, Kd*65536, Kp*65536, Ki*65536, QPPS, CRC16]
"""

import serial, struct, time, sys

PORT  = '/dev/ttyO5'
BAUD  = 460800
ADDR  = 0x80

# ── Tune these ────────────────────────────────────────────────────────────────
KP    = 1.0
KI    = 0.5
KD    = 0.25
QPPS  = 2500   # match qpps_max in pidconfig.txt
# ─────────────────────────────────────────────────────────────────────────────

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def send_acked(ser, payload):
    crc = crc16(payload)
    packet = payload + struct.pack('>H', crc)
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.05)
    ack = ser.read(1)
    ok = len(ack) == 1 and ack[0] == 0xFF
    return ok, ack.hex() if ack else 'none'

def encode_vpid(addr, cmd, kp, ki, kd, qpps):
    # RoboClaw encodes floats as uint32 = float * 65536
    kp_i = int(kp * 65536)
    ki_i = int(ki * 65536)
    kd_i = int(kd * 65536)
    return struct.pack('>BBIIIII', addr, cmd, kd_i, kp_i, ki_i, qpps, 0)[:-4]
    # Note: pack gives us addr+cmd+kd+kp+ki+qpps without the trailing zero

def build_vpid(addr, cmd, kp, ki, kd, qpps):
    kp_i = int(kp * 65536)
    ki_i = int(ki * 65536)
    kd_i = int(kd * 65536)
    return struct.pack('>BBIIII', addr, cmd, kd_i, kp_i, ki_i, qpps)

ser = serial.Serial(PORT, BAUD, timeout=0.5)
time.sleep(0.1)

print(f"Setting velocity PID: Kp={KP} Ki={KI} Kd={KD} QPPS={QPPS}")
print()

# Set M1 velocity PID (cmd 28)
payload = build_vpid(ADDR, 28, KP, KI, KD, QPPS)
ok, ack = send_acked(ser, payload)
print(f"M1 SetVelocityPID (cmd 28): {'OK' if ok else 'FAIL'}  ack={ack}")

time.sleep(0.05)

# Set M2 velocity PID (cmd 29)
payload = build_vpid(ADDR, 29, KP, KI, KD, QPPS)
ok, ack = send_acked(ser, payload)
print(f"M2 SetVelocityPID (cmd 29): {'OK' if ok else 'FAIL'}  ack={ack}")

time.sleep(0.05)

# Save to NVM flash (cmd 94) so it survives power cycle
payload = struct.pack('>BB', ADDR, 94)
ok, ack = send_acked(ser, payload)
print(f"WriteNVM          (cmd 94): {'OK' if ok else 'FAIL'}  ack={ack}")

ser.close()
print()
print("Done. Start balance_bot and switch to mode 1.")
