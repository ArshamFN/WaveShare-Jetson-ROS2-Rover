# Session 001 — Jetson Setup, Remote Access & UART Debugging

**Date:** 2026-02-20
**Status:** 🔴 Blocked — Awaiting Waveshare Support Response

---

## Goal
Get the Jetson Orin Nano Super fully set up, establish reliable remote access,
and achieve working UART communication between the Jetson and the
Wave Rover's General Driver Board (ESP32).

---

## What Was Accomplished

### 1. Jetson Initial Setup
- Flashed JetPack 6.2.1 onto a 128GB Class 10 SD card using BalenaEtcher,
  following the official NVIDIA getting-started guide
- Confirmed firmware version: **36.4.4** (meets the 36.x minimum requirement)
- Completed first boot and initial system configuration
- Activated **SUPER mode** on the Jetson for enhanced performance
- Ran full system update

### 2. Remote Access Setup
- Established SSH connection to the Jetson from a Windows laptop using **PuTTY**
- Installed and configured **NoMachine** for reliable remote desktop access
- Both remote access methods confirmed working

### 3. Jetson-to-Rover Wiring (No Prior Documentation Existed)
- Successfully wired the Jetson Orin Nano Super to the Wave Rover's
  General Driver Board with no single existing guide for this specific combination
- Information was pieced together from multiple scattered sources
- Connection method: Jetson Nano Adapter (C) → 2x5 header cable →
  Outer 40-pin header on General Driver Board
- Serial port used: `/dev/ttyTHS1` (UARTA — pins 8 and 10 on Jetson 40-pin header)
- **Note:** This wiring process will be formally documented as a standalone guide,
  as no public documentation currently exists for this hardware combination.

### 4. ROS2 Humble Installation
- Began installation of ROS2 Humble on the Jetson via SSH
- Installation in progress at end of session

---

## Issue Encountered — Garbled UART Data

### Symptom
After establishing the physical connection, a passive listening script was run
on `/dev/ttyTHS1` to receive JSON telemetry from the rover's ESP32.
All received data was consistently garbled at every baud rate tested.

**Example output at 115200 baud:**
```
b'\x08\x15\x13\x00\x03\x01v\x00$\x00\x80\x00\x88\x00\x19\x00\x8e\x00\x80'
```

### Baud Rates Tested
- 115200 (officially documented rate) — garbled
- 57600 — garbled
- 9600 — garbled

### Troubleshooting Steps Taken
- Confirmed `/dev/ttyTHS1` is the correct and active port (ttyTHS2 returned empty)
- Verified TX→RX and RX→TX wiring is correct
- Enabled UARTA on pins 8 and 10 via `jetson-io.py`
- Disabled `nvgetty` (NVIDIA serial console service) and rebooted
- Granted `dialout` group permissions to user
- Set `RTS/DTR=False` in Python script to match Waveshare's official demo
- Disabled hardware flow control at both Python and kernel level via `stty -crtscts`
- Confirmed ESP32 is healthy — WiFi web interface at 192.168.4.1 controls rover normally
- Confirmed the Jetson IS receiving data (not silent) — data is present but unreadable

### What Was Ruled Out
- Wrong serial port
- Baud rate mismatch
- File permission issues
- ESP32 hardware failure
- Hardware flow control interference

### Current Hypothesis
Possible voltage level mismatch or hardware incompatibility between the
Jetson Nano Adapter (C) and the Jetson Orin Nano Super's UART implementation.
The adapter may have been designed for the original Jetson Nano and may not
be fully compatible with the Orin Nano Super's UART signal characteristics.

---

## Current Status
- Waveshare technical support ticket submitted
- Support ticket requests: exact pin mapping of the Jetson Nano Adapter (C),
  confirmation of voltage level handling, and correct baud rate for Orin Nano Super
- Session paused pending support response

---

## Next Steps
- [ ] Receive and review Waveshare support response
- [ ] Apply recommended fix and retest UART communication
- [ ] Complete ROS2 Humble installation
- [ ] Document Jetson-to-Rover wiring as a standalone hardware guide
```
