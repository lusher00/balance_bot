#!/usr/bin/env python3
"""RoboClaw braking-quadrant bench test.

Talks packet-serial straight to the controller: drives one motor (or both)
at +duty for `ms`, flips to -duty for `ms`, then coasts, sampling speed,
current, bus voltage and the error/warning word every 10 ms. Writes a CSV
and prints the numbers that answer "does the controller brake when the
duty opposes rotation?".

WHEELS OFF THE GROUND. The balance_bot service owns the UART, so:

    sudo systemctl stop balance_bot
    sudo python3 tools/bench_rev.py --motor 2 --duty 0.5 --ms 2000
    sudo python3 tools/bench_rev.py --motor 2 --duty 0.5 --ms 2000 --amps 15
    sudo systemctl start balance_bot

--amps changes the controller's max-current limit for the run only and
restores the previous value on exit (also on Ctrl-C). Motor 1 = right,
motor 2 = left, as wired on this bot. Nothing is written to NVM.
"""
import argparse, os, select, signal, struct, sys, termios, time

ADDR = 0x80

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

class Claw:
    def __init__(self, port, baud):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = 0; a[1] = 0; a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        b = getattr(termios, 'B%d' % baud)
        a[4] = b; a[5] = b
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.fails = 0

    def _rx(self, n, timeout=0.05):
        buf = b''
        end = time.monotonic() + timeout
        while len(buf) < n:
            left = end - time.monotonic()
            if left <= 0:
                break
            r, _, _ = select.select([self.fd], [], [], left)
            if r:
                try:
                    buf += os.read(self.fd, n - len(buf))
                except BlockingIOError:
                    pass
        return buf

    def write(self, cmd, payload=b''):
        pkt = bytes([ADDR, cmd]) + payload
        pkt += struct.pack('>H', crc16(pkt))
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, pkt)
        ack = self._rx(1)
        ok = ack == b'\xff'
        self.fails = 0 if ok else self.fails + 1
        return ok

    def read(self, cmd, n):
        hdr = bytes([ADDR, cmd])
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, hdr)
        rx = self._rx(n + 2)
        if len(rx) != n + 2 or crc16(hdr + rx[:n]) != struct.unpack('>H', rx[n:])[0]:
            self.fails += 1
            return None
        self.fails = 0
        return rx[:n]

    # --- commands used here ---
    def duty(self, motor, d):           # d in -1..1
        raw = int(max(-1.0, min(1.0, d)) * 32767)
        p = struct.pack('>h', raw)
        if motor == 1:   return self.write(32, p)
        if motor == 2:   return self.write(33, p)
        return self.write(34, p + p)      # both

    def speed(self, motor):             # qpps, signed
        r = self.read(18 if motor == 1 else 19, 5)
        return None if r is None else struct.unpack('>i', r[:4])[0]

    def currents(self):                 # amps (m1, m2); negative = regen
        r = self.read(49, 4)
        if r is None: return (None, None)
        a, b = struct.unpack('>hh', r)
        return (a / 100.0, b / 100.0)

    def vbatt(self):
        r = self.read(24, 2)
        return None if r is None else struct.unpack('>H', r)[0] / 10.0

    def error(self):
        r = self.read(90, 4)
        return None if r is None else struct.unpack('>I', r)[0]

    def get_max_current(self, motor):   # amps
        r = self.read(135 if motor == 1 else 136, 8)
        return None if r is None else struct.unpack('>I', r[:4])[0] / 100.0

    def set_max_current(self, motor, amps):
        p = struct.pack('>II', int(amps * 100), 0)
        return self.write(133 if motor == 1 else 134, p)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyS1')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--motor', default='2', choices=['1', '2', 'both'], help='1=right 2=left')
    ap.add_argument('--duty', type=float, default=0.5)
    ap.add_argument('--ms', type=int, default=2000, help='length of each phase')
    ap.add_argument('--amps', type=float, default=0.0, help='temporary max-current limit for this run (0 = leave as is)')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    duty = max(0.05, min(0.8, abs(args.duty)))
    ms = max(200, min(5000, args.ms))
    motor = 0 if args.motor == 'both' else int(args.motor)
    probe = 2 if motor == 0 else motor           # which speed register to sample
    out = args.out or 'bench_rev_%s_m%s_d%.2f_a%g.csv' % (time.strftime('%Y%m%d_%H%M%S'), args.motor, duty, args.amps)

    c = Claw(args.port, args.baud)
    saved = {m: c.get_max_current(m) for m in (1, 2)}
    v = c.vbatt()
    print('max current before: M1 %s A  M2 %s A   vbatt %s V' % (saved[1], saved[2], v))
    if saved[1] is None or v is None:
        print('no reply from the controller -- is balance_bot stopped? right port/baud?')
        sys.exit(1)

    def restore(*_):
        for _ in range(3):
            c.duty(0, 0.0)
        if args.amps > 0:
            for m in (1, 2):
                if saved[m]:
                    c.set_max_current(m, saved[m])
            print('max current restored: M1 %s A  M2 %s A' % (c.get_max_current(1), c.get_max_current(2)))
    signal.signal(signal.SIGINT, lambda *a: (restore(), sys.exit(1)))

    if args.amps > 0:
        for m in (1, 2):
            c.set_max_current(m, args.amps)
        print('max current for this run: M1 %s A  M2 %s A' % (c.get_max_current(1), c.get_max_current(2)))

    rows = []
    t0 = time.monotonic()
    phases = [('fwd', duty, ms / 1000.0), ('rev', -duty, ms / 1000.0), ('coast', 0.0, 0.6)]
    vb = v
    err = None
    n = 0
    for name, d, dur in phases:
        c.duty(motor, d)
        p0 = time.monotonic()
        while time.monotonic() - p0 < dur:
            tick = time.monotonic()
            sp = c.speed(probe)
            a1, a2 = c.currents()
            if n % 10 == 0:
                vb = c.vbatt()
                err = c.error()
            rows.append((tick - t0, name, d, sp, a1, a2, vb, err))
            n += 1
            if c.fails >= 5:
                print('serial failing, aborting'); break
            rem = 0.010 - (time.monotonic() - tick)
            if rem > 0: time.sleep(rem)
    restore()

    with open(out, 'w') as f:
        f.write('t,phase,duty,speed_qpps,amps_m1,amps_m2,vbatt,error_word\n')
        for r in rows:
            f.write('%.3f,%s,%.3f,%s,%s,%s,%s,%s\n' % (r[0], r[1], r[2],
                    '' if r[3] is None else r[3], '' if r[4] is None else '%.2f' % r[4],
                    '' if r[5] is None else '%.2f' % r[5], '' if r[6] is None else '%.1f' % r[6],
                    '' if r[7] is None else '0x%08X' % r[7]))
    print('wrote', out, '(%d samples)' % len(rows))

    # --- summary ---
    ai = 3 + probe                     # amps column for the probed motor (4=m1, 5=m2)
    fwd = [r for r in rows if r[1] == 'fwd' and r[3] is not None]
    rev = [r for r in rows if r[1] == 'rev' and r[3] is not None]
    coast = [r for r in rows if r[1] == 'coast' and r[3] is not None]
    if not fwd or not rev:
        print('not enough samples'); return
    v_end = fwd[-1][3]
    print('end of fwd phase: %d qpps, %.2f A' % (v_end, fwd[-1][ai] or 0))
    t_rev = rev[0][0]
    cross = next((r for r in rev if (r[3] == 0) or (r[3] * v_end < 0)), None)
    first = [r for r in rev if r[0] - t_rev < 0.2 and r[ai] is not None]
    mean_a = sum(r[ai] for r in first) / len(first) if first else float('nan')
    peak_a = max((abs(r[ai]) for r in first), default=float('nan'))
    print('reversal: speed crossed zero after %s   amps in first 200 ms: mean %.2f, peak %.2f'
          % ('%.0f ms' % ((cross[0] - t_rev) * 1000) if cross else 'NEVER within %d ms' % ms, mean_a, peak_a))
    if cross:
        decel_rev = abs(v_end) / (cross[0] - t_rev)
        print('  decel under reverse duty: %.0f qpps/s' % decel_rev)
    if len(coast) > 5:
        dv = abs(coast[0][3]) - abs(coast[-1][3])
        dt = coast[-1][0] - coast[0][0]
        if dt > 0 and coast[0][3] != 0:
            print('  decel coasting (duty 0):  %.0f qpps/s  (from %d qpps)' % (dv / dt, coast[0][3]))
    words = sorted({r[7] for r in rev if r[7]})
    if words:
        print('  error/warning words seen during reversal: %s' % ', '.join('0x%08X' % w for w in words))
    print('verdict: ' + ('BRAKES (reverse decel well above coast, amps present)' if cross and peak_a > 1.0 and (cross[0] - t_rev) < 0.15
                         else 'NO BRAKING in the opposing quadrant' if (not cross or (cross[0] - t_rev) > 0.3) and peak_a < 0.5
                         else 'inconclusive -- send the CSV'))

if __name__ == '__main__':
    main()
