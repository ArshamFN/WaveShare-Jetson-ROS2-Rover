#!/usr/bin/env python3
"""
UART Interactive Terminal — WAVE ROVER <-> Jetson Orin Nano Super
Sends JSON commands to the rover's ESP32 and prints responses in real time.
Run this after uart_sanity_check.py confirms the connection is working.

Usage:
    Usage: python3 uart_terminal.py [--port /dev/ttyTHS1] [--baud 115200]

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
import argparse
import threading

def read_serial(ser):
    while True:
        try:
            line = ser.readline()
            if line:
                try:
                    decoded = line.decode('utf-8').strip()
                    print(f"Received: {decoded}")
                except UnicodeDecodeError:
                    print(f"Received (raw): {line}")
        except serial.SerialException:
            break

def main():
    parser = argparse.ArgumentParser(description='UART Terminal for Wave Rover')
    parser.add_argument('--port', type=str, default='/dev/ttyTHS1', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        rtscts=True,
        timeout=2
    )

    print("Connected. Type JSON commands and press Enter. Ctrl+C to quit.\n")

    recv_thread = threading.Thread(target=read_serial, args=(ser,))
    recv_thread.daemon = True
    recv_thread.start()

    try:
        while True:
            command = input("")
            if command.strip():
                ser.write(command.encode() + b'\n')
    except KeyboardInterrupt:
        print("\nSending stop command...")
        ser.write(b'{"T":1,"L":0,"R":0}\n')
    finally:
        ser.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()
