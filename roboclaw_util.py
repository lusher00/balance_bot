#!/usr/bin/env python3
"""
roboclaw_util.py — RoboClaw packet serial utility
BBB: /dev/ttyO5 @ 460800, address 0x01

Usage:
  python3 roboclaw_util.py <command> [args]

Commands:
  status          — read firmware version, voltages, temps, error status
  encoders        — read encoder counts and speeds
  currents        — read motor currents
  pid1            — read velocity PID for M1
  pid2            — read velocity PID for M2
  setpid1 Kp Ki Kd qpps  — set velocity PID for M1 and save
  setpid2 Kp Ki Kd qpps  — set velocity PID for M2 and save
  stop            — send zero velocity to both motors
  estop_config    — configure S3/S4/S5 as e-stop inputs and save
  read_config     — dump raw config register
  save            — write settings to EEPROM
  monitor         — loop, print voltage/current/encoders every second
"""

import serial
import struct
import time
import sys
import argparse

# ── Connection ────────────────────────────────────────────────────────────────
PORT    = "/dev/ttyO5"
BAUD    = 460800
ADDRESS = 0x01
TIMEOUT = 0.1   # seconds

# ── Command numbers (from Basicmicro packet serial spec) ──────────────────────
CMD_M1FORWARD           = 0
CMD_M1BACKWARD          = 1
CMD_M2FORWARD           = 4
CMD_M2BACKWARD          = 5
CMD_MIXED_FWD           = 8
CMD_MIXED_BACK          = 9
CMD_MIXED_RIGHT         = 10
CMD_MIXED_LEFT          = 11
CMD_DRIVE_M1_SIGNED     = 32
CMD_DRIVE_M2_SIGNED     = 33
CMD_DRIVE_MIXED_SIGNED  = 34

CMD_READ_ENC1           = 16
CMD_READ_ENC2           = 17
CMD_READ_SPEED1         = 18
CMD_READ_SPEED2         = 19
CMD_RESET_ENCODERS      = 20
CMD_READ_VERSION        = 21
CMD_READ_MAIN_BATT      = 24
CMD_READ_LOGIC_BATT     = 25
CMD_READ_MOTOR_CURRENTS = 49
CMD_READ_TEMP           = 82
CMD_READ_TEMP2          = 83
CMD_READ_STATUS         = 90
CMD_READ_ERROR_STATUS   = 90   # same register

CMD_SET_M1_VELPID       = 28
CMD_SET_M2_VELPID       = 29
CMD_READ_M1_VELPID      = 55
CMD_READ_M2_VELPID      = 56

CMD_GET_CONFIG          = 99
CMD_SET_CONFIG          = 98
CMD_WRITE_SETTINGS      = 94   # write NVM / EEPROM

CMD_DRIVE_M1M2_SPEED    = 37

# Config register bit positions (from user manual)
# Bits 4:2 control S3/S4/S5 pin function
# 0b000 = default, 0b010 = E-Stop (latching), 0b011 = E-Stop (non-latching)
CONFIG_ESTOP_LATCH      = 0b010
CONFIG_ESTOP_NOLATCH    = 0b011
PIN_MODE_SHIFT          = 2    # bits [4:2]

# ── CRC ───────────────────────────────────────────────────────────────────────

def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

# ── Low-level send / recv ─────────────────────────────────────────────────────

def send_cmd(ser: serial.Serial, addr: int, cmd: int, payload: bytes = b"") -> None:
    packet = bytes([addr, cmd]) + payload
    crc = crc16(packet)
    ser.write(packet + struct.pack(">H", crc))

def read_bytes(ser: serial.Serial, n: int) -> bytes:
    data = b""
    deadline = time.time() + TIMEOUT * 10
    while len(data) < n and time.time() < deadline:
        chunk = ser.read(n - len(data))
        data += chunk
    return data

def check_ack(ser: serial.Serial) -> bool:
    b = read_bytes(ser, 1)
    return len(b) == 1 and b[0] == 0xFF

def send_recv(ser: serial.Serial, addr: int, cmd: int,
              payload: bytes = b"", recv_n: int = 0) -> bytes | None:
    """Send command, receive recv_n data bytes + 2 CRC bytes, validate CRC."""
    send_cmd(ser, addr, cmd, payload)
    if recv_n == 0:
        ok = check_ack(ser)
        return b"\xff" if ok else None
    total = recv_n + 2
    raw = read_bytes(ser, total)
    if len(raw) < total:
        return None
    data   = raw[:recv_n]
    rx_crc = struct.unpack(">H", raw[recv_n:recv_n+2])[0]
    # CRC covers address + cmd + data
    calc   = crc16(bytes([addr, cmd]) + data)
    if rx_crc != calc:
        print(f"  [WARN] CRC mismatch: got {rx_crc:#06x}, expected {calc:#06x}")
        return None
    return data

# ── Open serial ───────────────────────────────────────────────────────────────

def open_serial(port=PORT, baud=BAUD) -> serial.Serial:
    try:
        ser = serial.Serial(port, baud, timeout=TIMEOUT)
        time.sleep(0.05)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        sys.exit(1)

# ── Commands ──────────────────────────────────────────────────────────────────

def cmd_status(ser, addr):
    print("=== RoboClaw Status ===")

    # Firmware version
    send_cmd(ser, addr, CMD_READ_VERSION)
    raw = b""
    for _ in range(48):
        b = ser.read(1)
        if not b:
            break
        raw += b
        if b == b"\n":
            break
    version = raw.decode("ascii", errors="replace").strip()
    print(f"  Firmware : {version}")

    # Main battery
    data = send_recv(ser, addr, CMD_READ_MAIN_BATT, recv_n=2)
    if data:
        v = struct.unpack(">H", data)[0] / 10.0
        print(f"  Main bat : {v:.1f} V")

    # Logic battery
    data = send_recv(ser, addr, CMD_READ_LOGIC_BATT, recv_n=2)
    if data:
        v = struct.unpack(">H", data)[0] / 10.0
        print(f"  Logic bat: {v:.1f} V")

    # Temperature
    data = send_recv(ser, addr, CMD_READ_TEMP, recv_n=2)
    if data:
        t = struct.unpack(">H", data)[0] / 10.0
        print(f"  Temp     : {t:.1f} °C")

    # Status / error flags
    data = send_recv(ser, addr, CMD_READ_STATUS, recv_n=4)
    if data:
        flags = struct.unpack(">I", data)[0]
        print(f"  Status   : {flags:#010x}")
        if flags == 0:
            print("             OK — no faults")
        else:
            STATUS_BITS = {
                0: "Normal",
                1: "M1 OverCurrent Warning",
                2: "M2 OverCurrent Warning",
                3: "E-Stop",
                4: "Temperature Error",
                5: "Temperature2 Error",
                6: "Main Batt High Error",
                7: "Logic Batt High Error",
                8: "Logic Batt Low Error",
                9: "Main Batt High Warning",
               10: "Main Batt Low Warning",
               11: "Temperature Warning",
               12: "Temperature2 Warning",
               13: "M1 Home",
               14: "M2 Home",
            }
            for bit, name in STATUS_BITS.items():
                if flags & (1 << bit):
                    print(f"             [!] {name}")

def cmd_encoders(ser, addr):
    print("=== Encoders ===")
    for ch, cmd_enc, cmd_spd in [
        (1, CMD_READ_ENC1, CMD_READ_SPEED1),
        (2, CMD_READ_ENC2, CMD_READ_SPEED2),
    ]:
        data = send_recv(ser, addr, cmd_enc, recv_n=5)
        if data:
            count  = struct.unpack(">i", data[:4])[0]   # signed 32-bit
            status = data[4]
            print(f"  M{ch} count : {count:>12d}  (status={status:#04x})")
        data = send_recv(ser, addr, cmd_spd, recv_n=5)
        if data:
            speed  = struct.unpack(">i", data[:4])[0]
            status = data[4]
            print(f"  M{ch} speed : {speed:>12d} pps  (status={status:#04x})")

def cmd_currents(ser, addr):
    print("=== Motor Currents ===")
    data = send_recv(ser, addr, CMD_READ_MOTOR_CURRENTS, recv_n=4)
    if data:
        m1 = struct.unpack(">H", data[0:2])[0] / 100.0
        m2 = struct.unpack(">H", data[2:4])[0] / 100.0
        print(f"  M1: {m1:.2f} A")
        print(f"  M2: {m2:.2f} A")

def cmd_read_pid(ser, addr, channel):
    cmd = CMD_READ_M1_VELPID if channel == 1 else CMD_READ_M2_VELPID
    data = send_recv(ser, addr, cmd, recv_n=16)
    if data:
        p, i, d, qpps = struct.unpack(">IIII", data)
        kp = p / 65536.0
        ki = i / 65536.0
        kd = d / 65536.0
        print(f"=== M{channel} Velocity PID ===")
        print(f"  Kp   : {kp}")
        print(f"  Ki   : {ki}")
        print(f"  Kd   : {kd}")
        print(f"  QPPS : {qpps}")

def cmd_set_pid(ser, addr, channel, kp, ki, kd, qpps):
    cmd = CMD_SET_M1_VELPID if channel == 1 else CMD_SET_M2_VELPID
    p = int(kp * 65536)
    i = int(ki * 65536)
    d = int(kd * 65536)
    q = int(qpps)
    payload = struct.pack(">IIII", p, i, d, q)
    result = send_recv(ser, addr, cmd, payload=payload)
    if result:
        print(f"  M{channel} PID set: Kp={kp} Ki={ki} Kd={kd} QPPS={qpps}")
        cmd_save(ser, addr)
    else:
        print(f"  ERROR: set PID M{channel} failed")

def cmd_stop(ser, addr):
    print("=== Stopping Motors ===")
    # Drive both motors at speed 0
    payload = struct.pack(">ii", 0, 0)
    result = send_recv(ser, addr, CMD_DRIVE_M1M2_SPEED, payload=payload)
    if result:
        print("  Both motors stopped.")
    else:
        # Fallback: individual signed drive
        send_recv(ser, addr, CMD_DRIVE_M1_SIGNED, payload=struct.pack(">i", 0))
        send_recv(ser, addr, CMD_DRIVE_M2_SIGNED, payload=struct.pack(">i", 0))
        print("  Stop sent (fallback).")

def cmd_read_config(ser, addr):
    print("=== Config Register ===")
    data = send_recv(ser, addr, CMD_GET_CONFIG, recv_n=2)
    if data:
        cfg = struct.unpack(">H", data)[0]
        print(f"  Raw config: {cfg:#06x}  ({cfg:016b}b)")
        # Decode known bits
        ctrl_mode = cfg & 0x07
        modes = {0: "Packet Serial", 1: "Analog", 2: "RC", 3: "Simple Serial"}
        print(f"  Control mode : {modes.get(ctrl_mode, ctrl_mode)}")
        pin_mode = (cfg >> PIN_MODE_SHIFT) & 0x07
        pin_modes = {
            0b000: "Default / disabled",
            0b001: "Home/Limit switches",
            0b010: "E-Stop (latching)",
            0b011: "E-Stop (non-latching)",
            0b100: "Voltage clamp output",
        }
        print(f"  S3/S4/S5 mode: {pin_modes.get(pin_mode, f'{pin_mode:#05b}')}")
        return cfg
    return None

def cmd_estop_config(ser, addr, latching=True):
    """Configure S3, S4, S5 as e-stop inputs."""
    print("=== Configuring E-Stop Pins ===")
    data = send_recv(ser, addr, CMD_GET_CONFIG, recv_n=2)
    if not data:
        print("  ERROR: could not read config")
        return
    cfg = struct.unpack(">H", data)[0]
    print(f"  Current config: {cfg:#06x}")

    # Clear bits [4:2] and set e-stop mode
    mode = CONFIG_ESTOP_LATCH if latching else CONFIG_ESTOP_NOLATCH
    cfg = (cfg & ~(0x07 << PIN_MODE_SHIFT)) | (mode << PIN_MODE_SHIFT)
    print(f"  New config    : {cfg:#06x}  ({'latching' if latching else 'non-latching'} e-stop)")

    payload = struct.pack(">H", cfg)
    result = send_recv(ser, addr, CMD_SET_CONFIG, payload=payload)
    if result:
        print("  Config written.")
        cmd_save(ser, addr)
    else:
        print("  ERROR: set config failed")

def cmd_save(ser, addr):
    result = send_recv(ser, addr, CMD_WRITE_SETTINGS)
    if result:
        print("  Settings saved to EEPROM.")
    else:
        print("  ERROR: save failed (or command not supported in this firmware)")

def cmd_monitor(ser, addr, interval=1.0):
    print("=== Monitor (Ctrl-C to stop) ===")
    print(f"{'Time':>6}  {'Vbat':>6}  {'Temp':>6}  {'M1A':>6}  {'M2A':>6}  {'Enc1':>12}  {'Enc2':>12}  {'Spd1':>8}  {'Spd2':>8}")
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0

            v_data = send_recv(ser, addr, CMD_READ_MAIN_BATT, recv_n=2)
            v = struct.unpack(">H", v_data)[0] / 10.0 if v_data else float("nan")

            t_data = send_recv(ser, addr, CMD_READ_TEMP, recv_n=2)
            temp = struct.unpack(">H", t_data)[0] / 10.0 if t_data else float("nan")

            c_data = send_recv(ser, addr, CMD_READ_MOTOR_CURRENTS, recv_n=4)
            if c_data:
                m1a = struct.unpack(">H", c_data[0:2])[0] / 100.0
                m2a = struct.unpack(">H", c_data[2:4])[0] / 100.0
            else:
                m1a = m2a = float("nan")

            e1_data = send_recv(ser, addr, CMD_READ_ENC1, recv_n=5)
            enc1 = struct.unpack(">i", e1_data[:4])[0] if e1_data else 0

            e2_data = send_recv(ser, addr, CMD_READ_ENC2, recv_n=5)
            enc2 = struct.unpack(">i", e2_data[:4])[0] if e2_data else 0

            s1_data = send_recv(ser, addr, CMD_READ_SPEED1, recv_n=5)
            spd1 = struct.unpack(">i", s1_data[:4])[0] if s1_data else 0

            s2_data = send_recv(ser, addr, CMD_READ_SPEED2, recv_n=5)
            spd2 = struct.unpack(">i", s2_data[:4])[0] if s2_data else 0

            print(f"{t:6.1f}  {v:6.1f}  {temp:6.1f}  {m1a:6.2f}  {m2a:6.2f}  {enc1:12d}  {enc2:12d}  {spd1:8d}  {spd2:8d}")
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nMonitor stopped.")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="RoboClaw packet serial utility")
    parser.add_argument("command", help="Command to run (see module docstring)")
    parser.add_argument("args", nargs="*", help="Command arguments")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=ADDRESS)
    parser.add_argument("--no-latch", action="store_true",
                        help="Use non-latching e-stop (default: latching)")
    args = parser.parse_args()

    ser = open_serial(args.port, args.baud)
    addr = args.addr

    cmd = args.command.lower()

    if cmd == "status":
        cmd_status(ser, addr)
    elif cmd == "encoders":
        cmd_encoders(ser, addr)
    elif cmd == "currents":
        cmd_currents(ser, addr)
    elif cmd == "pid1":
        cmd_read_pid(ser, addr, 1)
    elif cmd == "pid2":
        cmd_read_pid(ser, addr, 2)
    elif cmd == "setpid1":
        if len(args.args) != 4:
            print("Usage: setpid1 Kp Ki Kd qpps")
            sys.exit(1)
        kp, ki, kd, qpps = map(float, args.args)
        cmd_set_pid(ser, addr, 1, kp, ki, kd, int(qpps))
    elif cmd == "setpid2":
        if len(args.args) != 4:
            print("Usage: setpid2 Kp Ki Kd qpps")
            sys.exit(1)
        kp, ki, kd, qpps = map(float, args.args)
        cmd_set_pid(ser, addr, 2, kp, ki, kd, int(qpps))
    elif cmd == "stop":
        cmd_stop(ser, addr)
    elif cmd == "estop_config":
        cmd_estop_config(ser, addr, latching=not args.no_latch)
    elif cmd == "read_config":
        cmd_read_config(ser, addr)
    elif cmd == "save":
        cmd_save(ser, addr)
    elif cmd == "monitor":
        interval = float(args.args[0]) if args.args else 1.0
        cmd_monitor(ser, addr, interval)
    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
        sys.exit(1)

    ser.close()

if __name__ == "__main__":
    main()
