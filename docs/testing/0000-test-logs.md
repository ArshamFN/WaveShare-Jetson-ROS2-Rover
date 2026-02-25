# Test Logs & Build Journal

## Purpose
This file serves as an index of all session logs. Each session has its own
dedicated file in this folder with full details.

---

## Session Index

| Session | Date | Title | Status |
|---------|------|-------|--------|
| 000 | 2026-02-16 | Project Kickoff & Parts Ordered | ✅ Complete |
| 001 | 2026-02-21 | Jetson Setup, Remote Access & UART Debugging | ✅ Complete |
| 002 | 2026-02-22 | UART Debugging & Breakthrough | ✅ Complete |
| 003 | 2026-02-22 | CAD Design — GRD Cover & RPLidar Mount | ✅ Complete |
| 004 | 2026-02-25 | Power Debugging, LiDAR Integration & ROS2 Motor Control | ✅ Complete |

---

## Session 000 — 2026-02-16: Project Kickoff & Parts Ordered
**Goal:** Finalize platform choice, create GitHub repository, and order all components.

**Summary:** Platform research completed and all hardware decisions finalized.
Repository created and documentation structure set up. All components ordered
for ~$760 CAD total. No issues encountered.

---

## Session 001 — 2026-02-21: Jetson Setup, Remote Access & UART Debugging
**Goal:** Set up the Jetson Orin Nano Super, establish remote access,
and achieve working UART communication with the Wave Rover.

**Summary:** Jetson successfully flashed, configured, and updated to JetPack 6.2.1.
Remote access established via PuTTY (SSH) and NoMachine (desktop). ROS2 Humble
installed successfully. Rover wired to Jetson — no existing documentation for this
hardware combination, so the process was pieced together from multiple sources.
UART communication established but all received data was garbled. Continued in Session 002.

**→ [Full session log](2026-02-20-session-001-jetson-setup-uart-debugging.md)**

---

## Session 002 — 2026-02-22: UART Debugging & Breakthrough
**Goal:** Identify root cause of garbled UART data and establish working
communication with the Wave Rover.

**Summary:** Two compounding hardware issues identified and resolved. First, the
Jetson Nano Adapter (C) was inserted upside down, holding the ESP32's boot pin
at the wrong voltage. Second, the Jetson Orin Nano has a known ttyTHS1 data
corruption bug that requires RTS/CTS hardware flow control. Fixed by enabling
uarta-cts/rts in jetson-io and adding a jumper between pins 11 and 36. UART
communication fully working — rover moves on command from the Jetson. ✅

**→ [Full session log](2026-02-22-session-002-UART-breakthrough.md)**

---

## Session 003 — 2026-02-22: CAD Design — GRD Cover & RPLidar Mount

**Goal:** Finalize CAD designs for the custom GRD electronics cover and
RPLidar C1 mounting case while waiting for antenna and standoff spacers to arrive.

**Summary:** Both CAD designs completed in Fusion 360. The GRD cover is a
functional redesign of the rover's original plastic electronics bay cap, adding
openings for the 40-pin header, improved OLED visibility, and WiFi antenna cable
routing. The RPLidar C1 mount was designed from scratch to secure the sensor
to the rover. No physical assembly this session — parts still in transit.

**→ [Full session log](2026-02-22-session-003-cad-grd-cover-lidar-mount.md)**

---

## Session 004 — 2026-02-25: Power Debugging, LiDAR Integration & ROS2 Motor Control
**Goal:** Resolve power system fault from first full assembly boot, integrate the
RPLidar C1 into ROS2, and achieve complete rover motor control through the `/cmd_vel`
topic.
**Summary:** UPS power fault diagnosed and resolved — root cause was the Jetson's
Type-C standby draw overloading the 5V buck converter. Power architecture redesigned
to route both boards through the BAT rail, reserving the 5V output for peripherals.
Runtime analysis completed for the BENKIA 18650 pack across all Jetson power modes —
15W mode recommended for SLAM sessions (~56 min runtime). DisplayPort emulator ordered
to resolve headless NoMachine GPU issue. Persistent udev symlinks established for both
USB serial devices (`/dev/lidar`, `/dev/rover`). RPLidar C1 driver built from source
and confirmed publishing live `/scan` data. Rover communication switched to USB serial
for reliability. `rover_driver` ROS2 node written and deployed — full end-to-end motor
control via `/cmd_vel` confirmed. ✅
**→ [Full session log](2026-02-24-session-004-power-architecture-LiDAR-integration-ROS2-motor-control.md)**

```
