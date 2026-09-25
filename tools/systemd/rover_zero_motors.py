#!/usr/bin/env python3
"""Send a zero wheel-speed command to the rover motor board."""

import sys
import time

import serial

PORT = '/dev/rover'
BAUD = 115200
ZERO = b'{"T":1,"L":0,"R":0}\n'


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1, write_timeout=1)
    except (serial.SerialException, OSError) as exc:
        print(f'rover_zero_motors: cannot open {PORT}: {exc}', file=sys.stderr)
        return 0
    try:
        ser.write(ZERO)
        ser.flush()
        time.sleep(0.05)
        ser.write(ZERO)
        ser.flush()
    except (serial.SerialException, OSError) as exc:
        print(f'rover_zero_motors: write to {PORT} failed: {exc}', file=sys.stderr)
    finally:
        ser.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
