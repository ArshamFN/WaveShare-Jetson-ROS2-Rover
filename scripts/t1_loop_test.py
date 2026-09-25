#!/usr/bin/env python3
"""
Open-loop vs closed-loop test for the MFD's T:1 command.
Bringup must NOT be running: this script owns the serial port.
Default: wheels-in-the-air sweep.  --ground: one 0.2 run on the floor.
"""
import json, statistics, sys, time
import serial

PORT, BAUD = '/dev/rover', 115200
AIR_LEVELS    = [0.1, 0.2, 0.3, 0.5, 0.7, 1.0]
GROUND_LEVELS = [0.2]
RUN_S, MEASURE_S, REST_S = 3.0, 1.5, 2.5


def send(s, l, r):
    s.write((json.dumps({'T': 1, 'L': l, 'R': r}) + '\n').encode())


def drive(s, val, seconds, measure):
    buf, Ls, Rs, volts = b'', [], [], None
    t0 = time.monotonic()
    next_send = 0.0
    while True:
        now = time.monotonic() - t0
        if now >= seconds:
            break
        if now >= next_send:
            send(s, val, val)
            next_send += 0.05
        buf += s.read(s.in_waiting or 1)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            try:
                d = json.loads(line)
            except Exception:
                continue
            if d.get('T') != 1001:
                continue
            if 'v' in d:
                volts = d['v'] / 100.0
            if measure and now >= seconds - MEASURE_S:
                Ls.append(float(d.get('L', 0)))
                Rs.append(float(d.get('R', 0)))
    return Ls, Rs, volts


def main():
    ground = '--ground' in sys.argv
    levels = GROUND_LEVELS if ground else AIR_LEVELS
    s = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2.0)
    s.reset_input_buffer()
    print(f"Mode: {'GROUND' if ground else 'WHEELS IN AIR'}\n")
    print(f"{'cmd':>5} {'L avg':>7} {'L sd':>6} {'R avg':>7} {'R sd':>6} {'L/cmd':>6} {'R/cmd':>6} {'volts':>6}")
    try:
        for v in levels:
            Ls, Rs, volts = drive(s, v, RUN_S, measure=True)
            drive(s, 0, REST_S, measure=False)
            if not Ls:
                print(f'{v:5.2f}  no T:1001 feedback received')
                continue
            la, ra = statistics.mean(Ls), statistics.mean(Rs)
            lsd = statistics.pstdev(Ls)
            rsd = statistics.pstdev(Rs)
            vs = f'{volts:.2f}' if volts else '?'
            print(f'{v:5.2f} {la:7.3f} {lsd:6.3f} {ra:7.3f} {rsd:6.3f} '
                  f'{la / v:6.2f} {ra / v:6.2f} {vs:>6}', flush=True)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        for _ in range(10):
            send(s, 0, 0)
            time.sleep(0.05)
        s.close()
        print('\nStopped.')


if __name__ == '__main__':
    main()
