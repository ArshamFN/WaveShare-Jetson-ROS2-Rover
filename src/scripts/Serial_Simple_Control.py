#!/usr/bin/env python3
"""
UART Interactive Terminal — WAVE ROVER <-> Jetson Orin Nano Super
Sends JSON commands to the rover's ESP32 and prints responses in real time.
Run this after uart_sanity_check.py confirms the connection is working.

Usage:
    python3 uart_terminal.py
    python3 uart_terminal.py --port /dev/ttyTHS2 --baud 9600

Example commands:
    {"T":1,"L":0.2,"R":0.2}   — move forward
    {"T":1,"L":-0.2,"R":-0.2} — move backward
    {"T":1,"L":0.2,"R":-0.2}  — turn right
    {"T":1,"L":-0.2,"R":0.2}  — turn left
    {"T":1,"L":0,"R":0}        — stop

Press Ctrl+C to exit.

{"T":1,"L":0.2,"R":0.2}
T — Command Type
This tells the ESP32 what kind of command you're sending. T:1 means "set motor speeds." The WAVE ROVER supports many different T values for different functions, for example T:114 controls the LEDs, T:126 requests IMU data, and so on. Think of it like a function number.
L — Left side speed
Controls the two motors on the left side of the rover. The value ranges from -0.5 to +0.5, where 0.5 is 100% forward, -0.5 is 100% backward, and 0 is stopped.
R — Right side speed
Same as L but for the right side motors.

"""

import serial
import threading
import argparse
import sys

def read_loop(ser):
    """Continuously reads and prints data from the rover."""
    while True:
        try:
            line = ser.readline()
            if line:
                print(f"ROVER: {line.decode('utf-8', errors='replace').strip()}")
        except Exception:
            break

def main():
    parser = argparse.ArgumentParser(description='UART interactive terminal for WAVE ROVER')
    parser.add_argument('--port', default='/dev/ttyTHS1', help='Serial port (default: /dev/ttyTHS1)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1, dsrdtr=None)
        ser.setRTS(False)
        ser.setDTR(False)
        ser.flushInput()
    except Exception as e:
        print(f"ERROR: Could not open port — {e}")
        print("Tip: Run uart_sanity_check.py first to diagnose the connection.")
        sys.exit(1)

    print("Connected! Type a JSON command and press Enter to send.")
    print("Press Ctrl+C to exit.\n")

    # Start background thread to print incoming rover data
    t = threading.Thread(target=read_loop, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            cmd = input("CMD: ").strip()
            if not cmd:
                continue
            if not cmd.endswith('\n'):
                cmd += '\n'
            ser.write(cmd.encode('utf-8'))
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Always stop the rover before closing
        try:
            ser.write(b'{"T":1,"L":0,"R":0}\n')
            print("Sent stop command to rover.")
        except Exception:
            pass
        ser.close()

if __name__ == '__main__':
    main()
