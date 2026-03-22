import serial, struct, time

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
    return crc & 0xFFFF

ser = serial.Serial('/dev/ttyO5', 460800, timeout=1)
time.sleep(0.1)

def send_cmd(buf):
    crc = crc16(buf)
    ser.write(buf + struct.pack('>H', crc))
    time.sleep(0.05)

def read_speed(cmd):
    buf = bytes([0x80, cmd])
    ser.write(buf)
    time.sleep(0.05)
    data = ser.read(7)
    if len(data) >= 5:
        speed = struct.unpack('>i', data[0:4])[0]
        status = data[4]
        return speed, status
    return None, None

for motor, duty_cmd, speed_cmd in [('M1', 0, 18), ('M2', 4, 19)]:
    print(f"Spinning {motor}...")
    send_cmd(bytes([0x80, duty_cmd, 127]))
    time.sleep(2)
    speed, status = read_speed(speed_cmd)
    print(f"{motor} QPPS: {speed}  status: {status}")
    send_cmd(bytes([0x80, duty_cmd, 0]))
    time.sleep(1)

ser.close()
