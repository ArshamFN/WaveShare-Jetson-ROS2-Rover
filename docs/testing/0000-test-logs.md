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
```
