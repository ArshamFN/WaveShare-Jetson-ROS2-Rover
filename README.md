# Autonomous Rover - Jetson + ROS2 Navigation

A 4WD autonomous ground robot built on the Waveshare UGV02 platform with NVIDIA Jetson Orin Nano Super, implementing ROS2 Humble navigation and SLAM.

![Project Status](https://img.shields.io/badge/status-in%20progress-yellow)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/platform-Jetson%20Orin%20Nano-green)

## Project Overview

**Goal:** Build a production-ready autonomous navigation system from scratch to demonstrate ROS2 development skills.

**Why This Project:**
- Master ROS2 architecture (nodes, topics, services, actions)
- Implement SLAM-based mapping and localization
- Deploy autonomous waypoint navigation with Nav2
- Document real engineering problem-solving
- Build an AI-ready platform for future computer vision integration

![UGV02 fully assembled with migrated hardware](images/testing/session-006-migration/session-006-UGV02-front.jpg)

## Hardware Platform

### Core Components
- **Platform:** Waveshare UGV02 (4WD skid-steer chassis)
- **Compute:** NVIDIA Jetson Orin Nano Super Developer Kit (8GB RAM, 67 TOPS AI)
- **Sensors:** Slamtec RPLidar C1 (12m range, 10Hz, TOF technology)
- **Power:** 3× 18650 cells in series via WaveShare UPS Module 3S
- **Motor Control:** Multi-Functional Driver (MFD) board via USB serial (`/dev/ttyACM0`)

### Technical Specifications

| Component | Specification |
|-----------|---------------|
| **Drivetrain** | 4× DCGM-370-12V-EN-333RPM motors, skid-steer kinematics |
| **Encoder Resolution** | ~20 PPR (front wheels only; rear wheels unencoded) |
| **Motor Stall Torque** | ~5.0 kg·cm |
| **CPU** | 6-core ARM Cortex-A78AE @ 1.7GHz |
| **GPU** | 512 CUDA cores + Tensor cores (67 TOPS) |
| **RAM** | 8GB LPDDR5 |
| **LiDAR Range** | 12m max (white objects); 6m (black objects) |
| **LiDAR Scan Rate** | 5kHz sample rate @ 10Hz rotation |
| **LiDAR Angular Resolution** | 0.72° typical |
| **Light Resistance** | 40,000 lux |
| **Battery Pack** | 3S 18650, 11.1V nominal, 12.6V full charge |

### Power Architecture

| Component | Power Source |
|-----------|-------------|
| Jetson Orin Nano Super | BAT rail (9–12.6V) via 5.5mm barrel jack |
| MFD Board | BAT rail (9–12.6V) direct |
| RPLidar C1 | Jetson Orin Nano Super USB port |

### Runtime Estimates (15W Jetson mode, 25% real-world derating applied)

| Motor Load | Estimated Runtime |
|------------|------------------|
| Cruising | ~56 min |
| Under load | ~39 min |

15W mode is the operational standard for SLAM sessions — it balances compute performance
with sufficient runtime for a full mapping run.

## Software Stack

- **Operating System:** Ubuntu 22.04 LTS (JetPack 6.2.1)
- **ROS Distribution:** ROS2 Humble Hawksbill
- **Navigation Framework:** Nav2 (Navigation2 stack)
- **SLAM Algorithm:** slam_toolbox (synchronous mapping mode, CeresSolver)
- **Visualization:** RViz2
- **Development Languages:** Python 3.10, C++17

## Current Status

**Last Updated:** March 9, 2026

The rover is fully assembled and operational with a complete ROS2 navigation stack. Gyrodometry
is live — the `rover_driver` node fuses the MFD's onboard gyroscope for heading with
encoder-averaged linear displacement, eliminating the need for a track width constant entirely.
A heading hold PD controller keeps the rover tracking straight during SLAM runs, a software
velocity ramp prevents hard-acceleration current spikes, and Zero Velocity Update (ZUPT)
corrects gyro bias drift continuously throughout mapping sessions.

### Completed Milestones

**Hardware:**
- ✅ Full mechanical assembly with custom 3D-printed MFD cover and RPLidar mount
- ✅ Power architecture designed and validated — both boards on BAT rail
- ✅ Persistent USB device symlinks: `/dev/lidar` (RPLidar C1), `/dev/rover` (MFD board)
- ✅ Platform migration from Wave Rover to UGV02 (motivated by encoder availability)

**Software:**
- ✅ JetPack 6.2.1 flashed and configured
- ✅ ROS2 Humble installed and verified
- ✅ Remote access via PuTTY (SSH) and NoMachine (desktop)
- ✅ RPLidar C1 driver built from source — live `/scan` topic confirmed
- ✅ `rover_driver` ROS2 node — full `/cmd_vel` to MFD JSON bridge operational
- ✅ `robot_description` package — URDF with accurate `base_link → laser` transform (0.1685m z offset)
- ✅ SLAM Toolbox configured in synchronous mapping mode with CeresSolver
- ✅ Encoder odometry calibrated: `TRACK_WIDTH = 0.0456 m` (gyroscope-referenced, 3-phase calibration script)
- ✅ Gyrodometry implemented: gyroscope (`gz`) for heading, encoder average `(odl + odr) / 2` for linear displacement — `TRACK_WIDTH` eliminated from odometry entirely
- ✅ First proper SLAM map produced with gyrodometry active
- ✅ Software velocity ramp implemented — linear (0.8 m/s²) and angular (2.0 rad/s²) axes
- ✅ Forward-only heading hold PD controller implemented — gyro-based straight-line correction with settle gate, deadband, spike clamp, and output cap
- ✅ Zero Velocity Update (ZUPT) implemented — continuous gyro bias correction during stationary pauses
- ✅ SLAM Toolbox parameters tuned — full 10 Hz scan ingestion, extended loop closure search radius (15 m)
- ✅ Best map quality to date — three clean walls over two full perimeter laps

### Known Hardware Notes

- **MFD encoder wiring swap:** The `odl`/`odr` odometry fields in the MFD's T:1001 feedback
  packet are physically reversed — left and right encoder readings are swapped on the board.
  All odometry code accounts for this swap.
- **Front-wheel-only encoding:** Only the front axle motors carry encoders. Rear wheels are
  passive and unencoded. Odometry is computed from front wheel data only.
- **Battery sag under hard acceleration:** Commanding all four motors from a standstill at
  full speed causes a simultaneous peak current draw that can sag the battery below the
  Jetson's minimum operating voltage, triggering a protective shutdown. A software velocity
  ramp in `rover_driver` limits acceleration to 0.8 m/s² and eliminates the shutdown under
  normal operation.
- **ttyTHS1 UART bug:** The Jetson Orin Nano has a known data corruption bug on `/dev/ttyTHS1`
  requiring RTS/CTS hardware flow control. See Session 002 log for the full fix.
  Rover communication uses the MFD's dedicated USB serial port (`/dev/rover`) for reliability.
- **UPS power rail routing:** The Jetson must be powered from the BAT rail via barrel jack,
  not the 5V regulated output. The 5V buck converter cannot handle the combined load of
  the MFD board and Jetson simultaneously.

## ROS2 Packages

### rover_driver
A ROS2 Python node that bridges the standard `/cmd_vel` topic to the UGV02 MFD board's
JSON-over-serial protocol. Subscribes to `geometry_msgs/Twist`, applies skid-steer
kinematics to compute differential wheel speeds, and writes JSON commands to `/dev/rover`.
Implements gyrodometry — fusing the MFD's onboard gyroscope (`gz`) for heading with the
encoder average `(odl + odr) / 2` for linear displacement — and publishes `nav_msgs/Odometry`
to `/odom` with the corresponding `odom → base_link` tf2 transform. Also implements a
software velocity ramp on both axes to prevent hard-acceleration current spikes, a
forward-only heading hold PD controller for straight-line drift correction, and Zero
Velocity Update (ZUPT) for continuous gyro bias correction during stationary pauses.

```bash
ros2 run rover_driver rover_driver_node
```

See [`src/ROS2/rover-driver/README.md`](src/ROS2/rover-driver/README.md) for full setup instructions.

### robot_description
A ROS2 package containing the rover's URDF and SLAM Toolbox configuration. The URDF
defines the `base_link → laser` transform at 0.1685m height, derived from physical
measurement. Includes the SLAM Toolbox launch file and `slam_toolbox.yaml` configuration.
SLAM Toolbox parameters have been tuned for full 10 Hz scan ingestion, reduced distance
penalty, and a 15 m loop closure search radius suited to indoor room mapping.

```bash
ros2 launch robot_description slam.launch.py
```

### rplidar_ros
Slamtec's official ROS2 LiDAR driver, built from source to include RPLidar C1 support.

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py
```

See [`src/ROS2/rplidar-ros/README.md`](src/ROS2/rplidar-ros/README.md) for full setup instructions.

## Documentation

- [Bill of Materials](docs/hardware/bill-of-materials.md) — Complete parts list with suppliers and costs
- [Test Logs & Build Journal](docs/testing/test-logs.md) — Full session-by-session build history

## Author

**Arsham Faghihnasiri**

Building autonomous systems and learning production ROS2 development.

- 📍 Greater Toronto Area, Ontario, Canada
- 💼 [LinkedIn](https://www.linkedin.com/in/arsham-faghihnasiri)
- 📧 arshamfaghihnasiri@gmail.com
- 🎓 Software Engineering

## Acknowledgments

- Waveshare for the UGV02 platform and MFD board
- NVIDIA for Jetson developer tools and documentation
- Slamtec for the RPLidar SDK and ROS2 driver
- The ROS2 and Nav2 open source communities

## License

MIT License — see [LICENSE](LICENSE) for details.
