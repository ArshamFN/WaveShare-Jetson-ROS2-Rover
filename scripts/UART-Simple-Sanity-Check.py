#!/usr/bin/env python3
"""
UART Sanity Check - Passive listener for verifying connection to Wave Rover.
Usage: python3 uart_sanity_check.py [--port /dev/ttyTHS1] [--baud 115200]
"""

import serial
import argparse
import time

def main():
    parser = argparse.ArgumentParser(description='UART Sanity Check for Wave Rover')
    parser.add_argument('--port', type=str, default='/dev/ttyTHS1', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")
    print("Listening for data from rover. Press Ctrl+C to stop.\n")

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            rtscts=True,
            timeout=2
        )

        no_data_count = 0

        while True:
            line = ser.readline()
            if line:
                no_data_count = 0
                print(f"Raw:     {line}")
                try:
                    decoded = line.decode('utf-8').strip()
                    print(f"Decoded: {decoded}")
                except UnicodeDecodeError:
                    print("Decoded: [could not decode as UTF-8]")
                print()
            else:
                no_data_count += 1
                if no_data_count == 3:
                    print("No data received. Check:")
                    print("  - Rover is powered on (OLED should be active)")
                    print("  - Adapter text-less side faces the driver board")
                    print("  - Pin 8 (TX) → board TX, Pin 10 (RX) → board RX, GND → GND")
                    print("  - Jumper wire between Jetson pin 11 and pin 36")
                    print()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
