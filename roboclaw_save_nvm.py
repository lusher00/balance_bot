#!/usr/bin/env python3
"""Save RoboClaw settings to NVM flash (2x7A unlock sequence)."""

import serial, struct, time

PORT = '/dev/ttyO5'
BAUD = 460800
ADDR = 0x80

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def send_acked(ser, payload):
    crc = crc16(payload)
    ser.reset_input_buffer()
    ser.write(payload + struct.pack('>H', crc))
    time.sleep(0.2)
    ack = ser.read(1)
    ok = len(ack) == 1 and ack[0] == 0xFF
    return ok, ack.hex() if ack else 'none'

ser = serial.Serial(PORT, BAUD, timeout=0.5)
time.sleep(0.1)

# WriteNVM with unlock bytes 0x55 0xAA
payload = struct.pack('>BBBB', ADDR, 94, 0x55, 0xAA)
ok, ack = send_acked(ser, payload)
print(f"WriteNVM cmd 94 + unlock: {'OK' if ok else 'FAIL'}  ack={ack}")

ser.close()
