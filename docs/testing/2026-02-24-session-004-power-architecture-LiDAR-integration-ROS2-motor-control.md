# Session 004 — Power Debugging, LiDAR Integration & ROS2 Motor Control

**Date:** 2026-02-25  
**Status:** ✅ Complete

---

## Goal

Complete mechanical assembly, resolve power system issues, get the RPLidar C1 publishing
scan data in ROS2, and achieve full rover motor control through the `/cmd_vel` topic.

---

## What Was Accomplished

1. Diagnosed and resolved a UPS power fault that prevented the system from booting
2. Redesigned the power architecture to route both boards through the BAT rail
3. Calculated real-world runtime estimates for the BENKIA 18650 battery pack
4. Ordered a DisplayPort emulator to resolve the headless NoMachine GPU issue
5. Set up persistent udev symlinks for both the LiDAR (`/dev/lidar`) and rover (`/dev/rover`)
6. Built `rplidar_ros` from source and confirmed live `/scan` topic data from the C1
7. Switched rover communication from 40-pin UART to USB serial for reliability
8. Wrote and deployed a ROS2 bridge node (`rover_driver`) that translates `/cmd_vel`
   Twist messages to the GRD's JSON protocol — confirmed full end-to-end motor control

---

## Issue 1 — UPS Power Fault on First Boot

### Symptom
On the first power-on with full assembly, the UPS made an oscillating/squelching noise
immediately after switching on. The noise persisted after unplugging the Jetson. Toggling
the power switch did not clear the fault — only physically removing the batteries reset it.

### Diagnosis
I isolated the fault by disconnecting components one at a time. The root cause was the
Jetson's Type-C port drawing standby current even though the Jetson cannot boot from USB-C.
This additional load pushed the UPS's SY8286 5V buck converter past its 5A limit. Once the
converter hit current limit, it oscillated, triggering the S-8254AA protection IC to latch
into a fault state. The latch can only be cleared by complete battery removal — the power
switch does not interrupt the protection IC's hold circuit.

The GRD board alone was already near the 5V rail's threshold. The Jetson's Type-C standby
draw pushed the system over the edge.

### Fix
I redesigned the power architecture to remove the 5V rail from the critical path:

| Component | Before | After |
|-----------|--------|-------|
| GRD board | BAT rail | BAT rail (unchanged) |
| Jetson Orin Nano Super | 5V rail via Type-C | BAT rail via 5.5mm/2.5mm barrel jack |
| LiDAR + peripherals | — | 5V/5A regulated output (reserved) |

The Jetson's carrier board accepts 9–20V DC input on the barrel jack, so the 9–12.6V BAT
rail is within spec. The 5V regulated output is now reserved for the LiDAR and other
low-power peripherals, well within its 5A rating.

---

## Power Budget & Runtime Analysis

With the new power architecture in place, I calculated real-world runtime estimates for
planning SLAM sessions.

### Battery Pack
- 3× BENKIA 18650 3500mAh cells in series
- Nominal voltage: 11.1V (3 × 3.7V)
- Full charge: 12.6V (3 × 4.2V)
- UPS cutoff: ~9V (3 × 3.0V)
- Rated energy: 38.85Wh

**BENKIA caveat:** BENKIA is a budget consumer brand with no published independent
discharge curves. At the 3–5A draw levels this system operates at, I estimated actual
deliverable capacity at 3000–3200mAh versus the rated 3500mAh. For future builds,
cells with published datasheets (Samsung 30Q, Sony VTC6, Molicel P28A) are recommended.
I applied a conservative 25% real-world derating factor to all estimates below.

### Component Power Draw

| Component | Idle | Typical | Peak |
|-----------|------|---------|------|
| Jetson Orin Nano Super (7W mode) | — | 7W | — |
| Jetson Orin Nano Super (15W mode) | — | 15W | — |
| Jetson Orin Nano Super (MAXN_SUPER) | — | 25W | — |
| WiFi antennas (2× 6dBi) | — | ~1W | — |
| 4× N20 motors | 2.6W | 6.5W | 17W |
| GRD board + ESP32 + sensors | — | ~1W | — |
| RPLidar C1 | — | 2.5W | — |

### Runtime Estimates

| Jetson Mode | Motor Load | Theoretical | Real-world (~25% derating) |
|-------------|------------|-------------|---------------------------|
| 7W | Cruising | 106 min | ~80 min |
| 7W | Under load | 66 min | ~50 min |
| 15W | Cruising | 74 min | ~56 min |
| 15W | Under load | 52 min | ~39 min |
| MAXN_SUPER | Cruising | 62 min | ~47 min |
| MAXN_SUPER | Under load | 46 min | ~35 min |

I chose 15W mode as the operational standard for SLAM sessions. It provides approximately
56 minutes of runtime under typical cruising loads — sufficient for a full mapping run —
while preserving enough headroom for burst loads during obstacle avoidance maneuvers.

---

## Hardware — DisplayPort Emulator

NoMachine only renders the full desktop environment when the Jetson's GPU detects an active
display via EDID. Without a physical monitor connected, the GPU initializes in a degraded
state and NoMachine shows only the NVIDIA logo.

I ordered a DisplayPort emulator ($16.06 CAD after tax) that presents a virtual EDID
to the Jetson, tricking it into full desktop initialization without a physical monitor.
This will eliminate the need for a physical monitor during all future sessions.

---

## Software — udev Persistent Symlinks

Both USB serial devices (LiDAR and GRD) use CP2102 adapters with the same vendor and
product IDs. Without udev rules, the OS assigns ttyUSB0 and ttyUSB1 arbitrarily at boot
depending on enumeration order, which would break ROS2 launch files and serial configs.

I assigned persistent symlinks using each device's unique hardware serial number:

**Rule file:** `/etc/udev/rules.d/99-rover.rules`

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="bc5c5bc70f64ef118e1fe0a9c169b110", SYMLINK+="lidar", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="ea837f39e100f0119c58c1295c2a50c9", SYMLINK+="rover", MODE="0666"
```

Result: `/dev/lidar` and `/dev/rover` are now permanent regardless of boot order or
reconnection sequence. All ROS2 configs reference these symlinks exclusively.

---

## Software — RPLidar C1 ROS2 Driver

The `rplidar_ros` package available via `apt` does not include C1 support — the C1 is a
newer DTOF model added to the repository after the apt package was released. I built from
source instead:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd rplidar_ros
git checkout ros2
cd ~/ros2_ws
colcon build
source install/setup.bash
```

The build produced a large number of compiler warnings from Slamtec's internal SDK code.
These are cosmetic — the build completed successfully and the warnings have no effect on
functionality.

**Verification:**
```bash
ros2 launch rplidar_ros rplidar_c1_launch.py
```

Output confirmed:
- Health status: OK
- Firmware: 1.02, Hardware Rev: 18
- Standard scan mode, 5kHz sample rate, 10Hz rotation, 16m max range

Live `/scan` topic data confirmed via `ros2 topic echo /scan --once`. The LiDAR is
publishing 360° distance measurements at 10Hz — exactly the input the SLAM algorithm
requires.

---

## Software — Rover Communication: USB over UART

Issues arose with the 40-pin UART connection that had worked in Session 002, so I decided
to switch to the GRD's dedicated USB Type-C port for a more reliable and consistent
communication path. The GRD has an onboard CP2102 specifically for this purpose, and USB
serial is inherently more robust than a hardware UART connection over a ribbon cable.

Communication confirmed clean with no corruption:

```python
ser = serial.Serial('/dev/rover', 115200, timeout=2)
ser.write(b'{"T":10031}\n')
# Response: b'{"T":10031}\n'  ← clean echo, no null bytes, no corruption
```

Motor control via direct Python also confirmed working before proceeding to ROS2.

---

## Software — rover_driver ROS2 Node

I wrote a ROS2 bridge node that connects the standard `/cmd_vel` topic to the GRD's JSON
protocol. The design is intentional: by subscribing to `/cmd_vel` and speaking
`geometry_msgs/Twist`, the rover becomes compatible with the entire ROS2 navigation
ecosystem — Nav2, teleoperation tools, and SLAM — without any modification.

**Kinematics conversion:**
```
left  = linear.x - angular.z × 0.5
right = linear.x + angular.z × 0.5
```

Both values are clamped to `[-1.0, 1.0]` and sent as `{"T":1,"L":x,"R":x}` JSON over
`/dev/rover`.

**End-to-end test:**

Terminal 1:
```bash
ros2 run rover_driver rover_driver_node
# [INFO] [rover_driver]: Rover driver node started, listening on /cmd_vel
```

Terminal 2:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

Result: rover moved forward. Motor control through ROS2 confirmed. ✅

---

## Repository Updates

Added `src/ROS2/` directory structure to the repository:

```
src/
  ROS2/
    rover_driver/    ← original ROS2 package with full source and setup README
    rplidar_ros/     ← setup README
```

Each package folder is self-contained with step-by-step setup documentation written for
reproducibility on a fresh Jetson.

---

## Known Issues / Notes

- The 40-pin UART connection (`/dev/ttyTHS1`) is physically intact and the device tree
  overlay is correctly configured, but communication is unreliable. Switched to USB as the
  primary communication channel. UART debugging is deferred to a future session if needed.
- BENKIA 18650 cells are budget-grade with no published discharge curves. Runtime estimates
  carry higher uncertainty than they would with name-brand cells.
- DisplayPort emulator not yet received — physical monitor still required for NoMachine
  desktop access during this session.
