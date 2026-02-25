# Autonomous Rover - Jetson + ROS2 Navigation

A 4WD autonomous ground robot built on the Waveshare Wave Rover platform with NVIDIA Jetson Orin Nano Super, implementing ROS2 Humble navigation and SLAM.

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

## Hardware Platform

### Core Components
- **Platform:** Waveshare Wave Rover (4WD skid-steer chassis)
- **Compute:** NVIDIA Jetson Orin Nano Super Developer Kit (8GB RAM, 67 TOPS AI)
- **Sensors:** Slamtec RPLidar C1 (16m range, 10Hz, DTOF technology)
- **Power:** 3× BENKIA 18650 3500mAh cells in series via WaveShare UPS Module 3S
- **Motor Control:** Onboard ESP32 (General Driver Board) via USB serial

### Technical Specifications

| Component | Specification |
|-----------|---------------|
| **Drivetrain** | 4× N20 motors, skid-steer kinematics |
| **CPU** | 6-core ARM Cortex-A78AE @ 1.7GHz |
| **GPU** | 512 CUDA cores + Tensor cores (67 TOPS) |
| **RAM** | 8GB LPDDR5 |
| **LiDAR Range** | 16m max (Standard scan mode) |
| **LiDAR Scan Rate** | 5kHz sample rate @ 10Hz rotation |
| **Light Resistance** | 30,000 lux (outdoor capable) |
| **Battery Pack** | 3S 18650, 11.1V nominal, 12.6V full charge |

### Power Architecture

| Component | Power Source |
|-----------|-------------|
| Jetson Orin Nano Super | BAT rail (9–12.6V) via 5.5mm barrel jack |
| General Driver Board (ESP32) | BAT rail (9–12.6V) direct |
| RPLidar C1 | Jetson Orin Nano Super USB Port |

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
- **SLAM Algorithm:** slam_toolbox
- **Visualization:** RViz2
- **Development Languages:** Python 3.10, C++17

## Current Status

**Last Updated:** February 25, 2026

The rover is fully assembled and operational. ROS2 motor control is confirmed working —
publishing to `/cmd_vel` drives the rover. The RPLidar C1 is publishing live 360° scan
data on the `/scan` topic. The platform is ready for SLAM configuration.

### Completed Milestones

**Hardware:**
- ✅ Full mechanical assembly with custom 3D printed GRD cover and RPLidar mount
- ✅ Power architecture designed and validated — both boards on BAT rail
- ✅ Persistent USB device symlinks: `/dev/lidar` (RPLidar C1), `/dev/rover` (GRD)

**Software:**
- ✅ JetPack 6.2.1 flashed and configured
- ✅ ROS2 Humble installed and verified
- ✅ Remote access via PuTTY (SSH) and NoMachine (desktop)
- ✅ RPLidar C1 driver built from source — live `/scan` topic confirmed
- ✅ `rover_driver` ROS2 node — full `/cmd_vel` to GRD JSON bridge operational

### Known Hardware Notes

- **ttyTHS1 UART bug:** The Jetson Orin Nano has a known data corruption bug on `/dev/ttyTHS1`
  requiring RTS/CTS hardware flow control. See Session 002 log for the full fix.
  Rover communication currently uses the GRD's dedicated USB serial port (`/dev/rover`)
  for reliability.
- **Jetson Nano Adapter (C) orientation:** The text-less side must face the GRD's 40-pin
  header. Reversed insertion holds the ESP32's boot pin at the wrong voltage.
- **UPS power rail routing:** The Jetson must be powered from the BAT rail via barrel jack,
  not the 5V regulated output. The 5V buck converter cannot handle the combined load of
  the GRD and Jetson simultaneously.

## ROS2 Packages

### rover_driver
A ROS2 Python node that bridges the standard `/cmd_vel` topic to the Wave Rover's
JSON-over-serial protocol. Subscribes to `geometry_msgs/Twist`, applies unicycle drive
kinematics to compute differential wheel speeds, and writes JSON commands to `/dev/rover`.

```bash
ros2 run rover_driver rover_driver_node
```

See [`src/ROS2/rover-driver/README.md`](src/ROS2/rover-driver/README.md) for full setup instructions.

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

- Waveshare for the Wave Rover platform
- NVIDIA for Jetson developer tools and documentation
- Slamtec for the RPLidar SDK and ROS2 driver
- The ROS2 and Nav2 open source communities

## License

MIT License — see [LICENSE](LICENSE) for details.
