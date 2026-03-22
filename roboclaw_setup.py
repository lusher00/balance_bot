""" PID, position PID, saves to NVM.

Velocity PID  (cmd 28/29): fixed-point scale = 65536
Position PID  (cmd 61/62): fixed-point scale = 1024
WriteNVM      (cmd 94):    key = 0xE22EAB7A, causes unit reset

Packet format velocity:  [addr, cmd, D*65536, P*65536, I*65536, QPPS, CRC16]
Packet format position:  [addr, cmd, D*1024,  P*1024,  I*1024,  MaxI*1024,
                          Deadzone, MinPos, MaxPos, CRC16]
"""

import serial, struct, time, sys

PORT = '/dev/ttyO5'
BAUD = 460800
ADDR = 0x80

# ── Velocity PID ─────────────────────────────────────────────────────────────
# Applied when using MIXEDSPEED (mode 1) or MIXEDSPEEDACCEL (mode 2).
# QPPS must match qpps_max in pidconfig.txt.
# Start conservative — increase Kp if motors feel sluggish.
VEL_KP   = 1.0
VEL_KI   = 0.5
VEL_KD   = 0.25
VEL_QPPS = 2500

# ── Position PID ──────────────────────────────────────────────────────────────
# Only used with position commands (cmd 65/66/67).
# Per manual starting point: P=2000, I=0, D=0 (in Motion Studio units).
# Velocity PID should be set to P=0,I=0,D=0 when using position mode.
# MaxI=0 disables integral windup cap (set to nonzero to limit).
# Deadzone in encoder ticks — 0 means no deadzone.
# MinPos/MaxPos = 0 means no limits.
SET_POSITION_PID = False   # ← set True to also configure position PID
POS_KP      = 2000.0
POS_KI      = 0.0
POS_KD      = 0.0
POS_MAXI    = 0.0
POS_DEADZONE = 0          # ticks
POS_MINPOS  = 0           # ticks (0 = no limit)
POS_MAXPOS  = 0           # ticks (0 = no limit)
# ─────────────────────────────────────────────────────────────────────────────

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def send_acked(ser, payload, timeout=0.5):
    crc = crc16(payload)
    ser.reset_input_buffer()
    ser.write(payload + struct.pack('>H', crc))
    time.sleep(timeout)
    ack = ser.read(1)
    ok = len(ack) == 1 and ack[0] == 0xFF
    return ok, ack.hex() if ack else 'none'

def fp(val, scale):
    return int(val * scale)

ser = serial.Serial(PORT, BAUD, timeout=1.0)
time.sleep(0.1)

print(f"=== RoboClaw Setup ===")
print(f"Port: {PORT}  Baud: {BAUD}  Addr: 0x{ADDR:02X}")
print()

# ── Velocity PID M1 (cmd 28) ─────────────────────────────────────────────────
print(f"Velocity PID: Kp={VEL_KP} Ki={VEL_KI} Kd={VEL_KD} QPPS={VEL_QPPS}")
payload = struct.pack('>BBIIII', ADDR, 28,
                      fp(VEL_KD, 65536), fp(VEL_KP, 65536),
                      fp(VEL_KI, 65536), VEL_QPPS)
ok, ack = send_acked(ser, payload)
print(f"  M1 SetVelocityPID (cmd 28): {'OK' if ok else 'FAIL'}  ack={ack}")

# ── Velocity PID M2 (cmd 29) ─────────────────────────────────────────────────
payload = struct.pack('>BBIIII', ADDR, 29,
                      fp(VEL_KD, 65536), fp(VEL_KP, 65536),
                      fp(VEL_KI, 65536), VEL_QPPS)
ok, ack = send_acked(ser, payload)
print(f"  M2 SetVelocityPID (cmd 29): {'OK' if ok else 'FAIL'}  ack={ack}")
print()

# ── Position PID M1/M2 (cmd 61/62) ───────────────────────────────────────────
if SET_POSITION_PID:
    print(f"Position PID: Kp={POS_KP} Ki={POS_KI} Kd={POS_KD} "
          f"MaxI={POS_MAXI} Deadzone={POS_DEADZONE} "
          f"MinPos={POS_MINPOS} MaxPos={POS_MAXPOS}")
    for motor, cmd in [('M1', 61), ('M2', 62)]:
        payload = struct.pack('>BBIIIIiII', ADDR, cmd,
                              fp(POS_KD, 1024), fp(POS_KP, 1024),
                              fp(POS_KI, 1024), fp(POS_MAXI, 1024),
                              POS_DEADZONE, POS_MINPOS, POS_MAXPOS)
        ok, ack = send_acked(ser, payload)
        print(f"  {motor} SetPositionPID (cmd {cmd}): {'OK' if ok else 'FAIL'}  ack={ack}")
    print()

# ── WriteNVM (cmd 94) — unit resets after this ───────────────────────────────
print("Saving to NVM (unit will reset)...")
payload = struct.pack('>BBI', ADDR, 94, 0xE22EAB7A)
ok, ack = send_acked(ser, payload, timeout=1.0)
print(f"  WriteNVM (cmd 94): {'OK' if ok else 'FAIL'}  ack={ack}")
if ok:
    print("  Unit resetting — wait 3s before restarting balance_bot")

ser.close()
print()
print("Done.")
