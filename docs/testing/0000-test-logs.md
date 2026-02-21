# Test Logs & Build Journal

## Purpose
This file serves as an index of all session logs. Each session has its own
dedicated file in this folder with full details.

---

## Session Index

| Session | Date | Title | Status |
|---------|------|-------|--------|
| 000 | 2026-02-16 | Project Kickoff & Parts Ordered | ✅ Complete |
| 001 | 2026-02-21 | Jetson Setup, Remote Access & UART Debugging | 🔴 Blocked — Awaiting Waveshare Support |

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
UART communication established but all received data is garbled at every baud rate
tested. All common causes ruled out. Leading hypothesis is a voltage-level
incompatibility with the Jetson Nano Adapter (C). Waveshare support ticket submitted.

**→ [Full session log](2026-02-20-session-001-jetson-setup-uart-debugging.md)**
