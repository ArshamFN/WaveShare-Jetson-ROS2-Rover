#!/usr/bin/env python3
"""
UART Sanity Check — WAVE ROVER <-> Jetson Orin Nano Super
Tests serial communication on /dev/ttyTHS1 (UARTA, pins 8 & 10).
Run this before attempting any ROS2 rover communication.

Usage:
    python3 uart_sanity_check.py
    python3 uart_sanity_check.py --port /dev/ttyTHS1

Here's everything it does:
1. Opens the serial port cleanly — uses the correct settings (dsrdtr=None, setRTS(False), setDTR(False)) that Waveshare's own demo code requires. A naive serial.Serial(port, 115200) call often fails with ESP32 devices because of these flags.
2. Handles errors gracefully — if the port doesn't exist or you don't have permission, it tells you clearly instead of crashing with a confusing Python traceback.
3. Passively listens for 3 seconds — doesn't send anything, just listens. This is the most reliable first test because it checks if the ESP32 is broadcasting at all, independent of whether your commands are formatted correctly.
4. Shows you both raw and decoded output — prints the raw bytes (repr(data)) so you can see exactly what's coming in, then attempts a UTF-8 decode so you can read it if it's valid JSON.
5. Gives you a diagnostic checklist if nothing comes back — instead of just printing an empty result, it tells you exactly what to check.
6. Accepts command line arguments — so you can test different ports and baud rates without editing the file, like python3 uart_sanity_check.py --port /dev/ttyTHS2 --baud 9600.

"""

import serial
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description='UART sanity check for WAVE ROVER')
    parser.add_argument('--port', default='/dev/ttyTHS1', help='Serial port (default: /dev/ttyTHS1)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")

    try:
        s = serial.Serial(args.port, args.baud, timeout=3, dsrdtr=None)
        s.setRTS(False)
        s.setDTR(False)
        s.flushInput()
    except Exception as e:
        print(f"ERROR: Could not open port — {e}")
        return

    print("Listening for data from rover ESP32 (3 seconds)...")
    time.sleep(1)
    data = s.read(500)
    s.close()

    if data:
        print(f"Received {len(data)} bytes:")
        print(repr(data))
        try:
            print("\nDecoded:")
            print(data.decode('utf-8'))
        except:
            print("(Could not decode as UTF-8 — possible baud rate or wiring issue)")
    else:
        print("No data received. Check:")
        print("  - Is the rover powered on?")
        print("  - Is the Jetson Nano Adapter (C) connected correctly?")
        print("  - Is nvgetty disabled? (sudo systemctl disable nvgetty)")
        print("  - Is UARTA enabled? (sudo /opt/nvidia/jetson-io/jetson-io.py)")

if __name__ == '__main__':
    main()
