#!/usr/bin/env python3
"""
In-place rotation test via T:1 (closed-loop wheel speed, m/s).
L = -cmd, R = +cmd (CCW). Reads wheel speeds and gyro gz.
Bringup must NOT be running. Rover on the floor, 0.3 m clear around it.
"""
import json, statistics, time
import serial

PORT, BAUD   = '/dev/rover', 115200
LEVELS       = [0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.25]
GZ_SCALE     = 0.001058   # rad/s per gz count (S009 calibration)
TRACK_WIDTH  = 0.174
RUN_S, MEASURE_S, REST_S, BIAS_S = 3.0, 1.5, 2.5, 2.0


def send(s, l, r):
    s.write((json.dumps({'T': 1, 'L': l, 'R': r}) + '\n').encode())


def drive(s, cmd, seconds, measure):
    buf, Ls, Rs, gzs = b'', [], [], []
    t0 = time.monotonic()
    next_send = 0.0
    while True:
        now = time.monotonic() - t0
        if now >= seconds:
            break
        if now >= next_send:
            send(s, -cmd, cmd)
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
            if measure and now >= seconds - MEASURE_S:
                Ls.append(float(d.get('L', 0)))
                Rs.append(float(d.get('R', 0)))
                gzs.append(float(d.get('gz', 0)))
    return Ls, Rs, gzs


def main():
    s = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2.0)
    s.reset_input_buffer()
    _, _, gz0 = drive(s, 0, BIAS_S + 0.5, measure=True)
    bias = statistics.mean(gz0) if gz0 else 0.0
    print(f'gz bias: {bias:.1f} counts\n')
    print(f"{'cmd':>5} {'L avg':>7} {'R avg':>7} {'|L|/cmd':>7} {'R/cmd':>6} "
          f"{'gyro':>6} {'ideal':>6} {'g/ideal':>7}")
    try:
        for c in LEVELS:
            Ls, Rs, gzs = drive(s, c, RUN_S, measure=True)
            drive(s, 0, REST_S, measure=False)
            if not Ls:
                print(f'{c:5.2f}  no feedback')
                continue
            la, ra = statistics.mean(Ls), statistics.mean(Rs)
            gyro = (statistics.mean(gzs) - bias) * GZ_SCALE
            ideal = 2.0 * c / TRACK_WIDTH
            print(f'{c:5.2f} {la:7.3f} {ra:7.3f} {abs(la) / c:7.2f} {ra / c:6.2f} '
                  f'{gyro:6.2f} {ideal:6.2f} {gyro / ideal:7.2f}', flush=True)
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
