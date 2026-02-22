# Session 002 – UART Debugging & Breakthrough

**Date:** 2026-02-22  
**Status:** ✅ Resolved  
**Continues from:** [Session 001 – Jetson Setup & Initial UART Attempts](./2026-02-20-session-001-jetson-setup-uart-debugging.md)

---

## Goals

- Identify root cause of garbled UART data between Jetson Orin Nano Super and Wave Rover ESP32
- Establish working serial communication over GPIO UART
- Send and receive JSON commands to/from the rover

---

## Accomplishments

- Identified two compounding hardware issues that caused all previous garbled data
- Established clean, reliable UART communication over `/dev/ttyTHS1`
- Confirmed bidirectional JSON communication with the rover
- Confirmed motor control via UART — **rover moves on command from the Jetson** ✅

---

## Root Causes Identified

### Issue 1: Jetson Nano Adapter (C) Inserted Upside Down

The Waveshare Jetson Nano Adapter (C) has no physical keying — both sides look nearly identical (only a small text label distinguishes them). When inserted with the **text side facing the driver board**, certain pins held the ESP32's boot mode line at the wrong voltage during power-on, preventing the ESP32 from starting. Symptoms included the OLED not turning on when the Jetson was connected, and the OLED freezing mid-boot if connected after power-on.

**Fix:** Insert the adapter with the **text-less side facing the driver board**.

---

### Issue 2: Jetson Orin Nano ttyTHS1 Hardware Data Corruption Bug

The Jetson Orin Nano has a known hardware/driver bug where `/dev/ttyTHS1` randomly inserts null bytes (`\x00`) and corrupts data unless RTS/CTS hardware flow control is enabled. This is documented in the NVIDIA Developer Forums:  
https://forums.developer.nvidia.com/t/nano-ttyths1-occasional-lost-bytes/273092

This bug caused every UART reading to appear as garbage — even with correct wiring, baud rate, and port. The loopback test (shorting pin 8 ↔ pin 10) confirmed the Jetson UART itself was corrupting data independently of the rover.

**Fix (two parts):**

1. Enable `uarta-cts/rts` pins via jetson-io:
```bash
   sudo /opt/nvidia/jetson-io/jetson-io.py
   # Navigate to: Configure header pins manually
   # Enable: uarta-cts/rts (11, 36)
   # Save and reboot
```

2. Since the Wave Rover driver board exposes no RTS/CTS pins, create an **RTS/CTS loopback** on the Jetson side by connecting a jumper wire between **pin 11 and pin 36** on the Jetson's 40-pin header. This satisfies the hardware handshake internally without requiring any changes to the rover.

3. Enable `rtscts=True` in all pyserial code:
```python
   ser = serial.Serial('/dev/ttyTHS1', 115200, rtscts=True, timeout=2)
```

---

## Troubleshooting Steps Taken (Chronological)

| Step | Finding |
|------|---------|
| Reviewed hardware docs (General Driver for Robots, UPS Module 3S, Jetson Orin Nano datasheet) | Identified 1.8V UART logic on Jetson — suspected voltage mismatch |
| Discovered Jetson Nano Adapter (C) handles 1.8V→3.3V conversion | Voltage mismatch ruled out |
| Reviewed Jetson Orin Nano UART datasheet section | Found 2 stop bits requirement — tested, did not fix issue |
| Tested all baud rates (9600, 19200, 38400, 57600, 115200) | Garbling consistent across all rates |
| Sent JSON command, listened for response | Still garbled — but confirmed data IS flowing |
| Attempted null-byte stripping | Remaining bytes still not valid JSON — not just a null-byte issue |
| Performed UART loopback test (pin 8 ↔ pin 10) | Loopback failed — confirmed Jetson UART itself corrupting data |
| Researched NVIDIA forums | Found known ttyTHS1 data corruption bug on Orin Nano |
| Enabled uarta-cts/rts in jetson-io, rebooted | Loopback test passed — `b'hello'` returned correctly |
| Discovered adapter orientation issue via GND-only test | Identified adapter upside down as ESP32 boot freeze cause |
| Reconnected with correct adapter orientation + RTS/CTS loopback | OLED boots, JSON response received, motor control confirmed ✅ |

---

## Working Configuration

| Parameter | Value |
|-----------|-------|
| Serial port | `/dev/ttyTHS1` |
| Baud rate | `115200` |
| RTS/CTS | `rtscts=True` |
| Stop bits | Default (1) |
| Jetson pin 11 | Jumpered to pin 36 (RTS/CTS loopback) |
| Adapter orientation | Text-less side facing driver board |
| TX | Jetson pin 8 → Driver board TX |
| RX | Jetson pin 10 → Driver board RX |
| GND | Jetson pin 6 or 9 → Driver board GND |

---

## Key Commands
```python
import serial, time

ser = serial.Serial('/dev/ttyTHS1', 115200, rtscts=True, timeout=2)
time.sleep(1)

# Send a command
ser.write(b'{"T":10031}\n')

# Read response
response = ser.read(200)
print(response)

ser.close()
```

Example motor control (forward 2 seconds, then stop):
```python
ser.write(b'{"T":1,"L":0.3,"R":0.3}\n')
time.sleep(2)
ser.write(b'{"T":1,"L":0,"R":0}\n')
```
