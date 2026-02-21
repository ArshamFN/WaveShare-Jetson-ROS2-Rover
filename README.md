# Autonomous Rover - Jetson + ROS2 Navigation

A 4WD autonomous ground robot built on the Waveshare Wave Rover platform with NVIDIA Jetson Orin Nano Super, implementing ROS2 Humble navigation and SLAM.

![Project Status](https://img.shields.io/badge/status-in%20progress-yellow)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/platform-Jetson%20Orin%20Nano-green)

## Project Overview

**Goal:** Build a production-ready autonomous navigation system from scratch to demonstrate ROS2 development skills for robotics engineering roles.

**Why This Project:**
- Master ROS2 architecture (nodes, topics, services, actions)
- Implement SLAM-based mapping and localization
- Deploy autonomous waypoint navigation with Nav2
- Document real engineering problem-solving for portfolio
- Build AI-ready platform for future computer vision integration

**Timeline:** 12-14 weeks from hardware assembly to autonomous navigation

## Hardware Platform

### Core Components
- **Platform:** Waveshare Wave Rover (4WD skid-steer chassis)
- **Compute:** NVIDIA Jetson Orin Nano Super Developer Kit (8GB RAM, 67 TOPS AI)
- **Sensors:** Slamtec RPLidar C1 (12m range, 10Hz, DTOF fusion technology)
- **Power:** 3x 18650 lithium batteries (7800mAh total) via Wave Rover UPS module
- **Motor Control:** Onboard ESP32 for low-level control with encoder feedback

### Technical Specifications

| Component | Specification |
|-----------|---------------|
| **Drivetrain** | 4x N20 motors with encoders, skid-steer kinematics |
| **CPU** | 6-core ARM Cortex-A78AE @ 1.7GHz |
| **GPU** | 512 CUDA cores + Tensor cores (67 TOPS) |
| **RAM** | 8GB LPDDR5 |
| **Lidar Range** | 12m (white objects), 6m (black objects) |
| **Lidar Scan Rate** | 5000 Hz @ 10Hz rotation frequency |
| **Light Resistance** | 30,000 lux (outdoor capable) |
| **Total Weight** | ~2.5kg |
| **Runtime** | 2-3 hours estimated |

## Software Stack

- **Operating System:** Ubuntu 22.04 LTS (JetPack 6.1)
- **ROS Distribution:** ROS2 Humble Hawksbill
- **Navigation Framework:** Nav2 (Navigation2 stack)
- **SLAM Algorithm:** slam_toolbox (pose-graph optimization)
- **Localization:** AMCL (Adaptive Monte Carlo Localization)
- **Visualization:** RViz2
- **Development Languages:** Python 3.10, C++17

## Project Phases

### ✅ Phase 0: Planning & Ordering (Week 1)
- [x] Research platform options
- [x] Select components and finalize BOM
- [x] Order all parts (~$760 CAD)
- [x] Create GitHub repository
- [x] Set up documentation structure

**Status:** COMPLETE - Parts arriving February 19, 2025

---

### Phase 1: Hardware Setup (Weeks 2-3)
- [x] Unbox and inventory components
- [x] Assemble Wave Rover base platform
- [ ] Mount Jetson Orin Nano on chassis
- [ ] Mount RPLidar C1 with custom bracket
- [x] Install batteries and test power distribution
- [x] Verify ESP32 motor control functionality
- [ ] Test encoder feedback

**Milestone:** Fully assembled rover with verified power and motor systems

---

### Phase 2: ROS2 Foundation (Weeks 3-4)
- [x] Flash JetPack 6.1 to Jetson
- [x] Install ROS2 Humble
- [ ] Configure ESP32-to-ROS2 serial communication bridge
- [ ] Write motor control node (velocity commands → ESP32)
- [ ] Implement odometry publisher (encoders → /odom topic)
- [ ] Test keyboard teleoperation
- [ ] Verify tf tree broadcasting

**Milestone:** Drive rover via ROS2 velocity commands, visualize odometry in RViz2

---

### Phase 3: Sensor Integration (Weeks 5-6)
- [ ] Install rplidar_ros2 driver package
- [ ] Configure Lidar parameters for RPLidar C1
- [ ] Visualize live scan data in RViz2
- [ ] Build complete TF tree (base_link → laser_frame)
- [ ] Fuse wheel odometry with Lidar data
- [ ] Test sensor stability and data quality

**Milestone:** Live 2D Lidar visualization with correct transforms

---

### Phase 4: SLAM Mapping (Weeks 7-8)
- [ ] Install and configure slam_toolbox
- [ ] Tune SLAM parameters (scan matching, loop closure)
- [ ] Create a map of the indoor test environment
- [ ] Test map quality and consistency
- [ ] Implement map save/load functionality
- [ ] Document mapping procedures

**Milestone:** Generate reliable 2D occupancy grid maps

---

### Phase 5: Autonomous Navigation (Weeks 9-12)
- [ ] Install Nav2 stack
- [ ] Configure global costmap (static map layer)
- [ ] Configure local costmap (obstacle detection)
- [ ] Set up path planner (NavFn or Smac Planner)
- [ ] Configure controller (DWB or TEB)
- [ ] Implement recovery behaviours
- [ ] Test single waypoint navigation
- [ ] Test multi-waypoint missions
- [ ] Tune navigation stack for performance

**Milestone:** Autonomous navigation from point A to point B with obstacle avoidance

---

## Skills Demonstrated

This project showcases professional robotics engineering competencies:

**ROS2 Development:**
- Node architecture and communication patterns
- Topic pub/sub, services, and actions
- Parameter management and launch files
- TF (transform) tree configuration

**Mobile Robotics:**
- Differential/skid-steer kinematics
- Wheel odometry and dead reckoning
- Sensor fusion techniques
- Path planning and trajectory control

**SLAM & Localization:**
- 2D Lidar-based mapping
- Pose-graph SLAM optimization
- Loop closure detection
- Particle filter localization (AMCL)

**System Integration:**
- Hardware-software interfacing
- Serial communication protocols
- Power management and safety
- Real-time performance optimization

**Professional Practices:**
- Technical documentation
- Systematic debugging methodology
- Iterative development process

## Future Enhancements (Project #2)

After achieving autonomous navigation, planned upgrades include:

- **Computer Vision:** Intel RealSense D435i depth camera
- **Visual Obstacle Avoidance:** YOLO object detection, semantic segmentation
- **Outdoor Navigation:** GPS waypoint following
- **Advanced Features:** Multi-robot coordination, payload delivery mechanism
- **Machine Learning:** Reinforcement learning for dynamic environments

## Documentation

- [Bill of Materials](docs/hardware/bill-of-materials.md) - Complete parts list with suppliers
- [Assembly Guide](docs/hardware/assembly-guide.md) - Step-by-step build instructions
- [Wiring Diagram](docs/hardware/wiring-diagram.md) - Electrical connections
- [Software Setup](docs/software/setup-guide.md) - ROS2 installation and configuration
- [System Architecture](docs/software/architecture.md) - Design decisions and rationale
- [Test Logs](docs/testing/test-logs.md) - Daily progress and problem-solving

## Demo Videos

*Demo videos will be added as milestones are completed*

## Current Status

**Last Updated:** February 17, 2025

**Current Phase:** Phase 0 Complete - Parts Ordered

**Investment:** ~$760 CAD total project cost

**Timeline:**
- Parts ordered: February 17, 2025
- Expected delivery: February 19, 2025
- Assembly start: February 19, 2025
- Projected completion: May 2025

**Next Immediate Actions:**
1. ✅ Components ordered (Wave Rover, Jetson, Lidar, batteries)
2. ✅ Install ROS2 Humble on development machine
3. ✅ Study Waveshare Wave Rover documentation
4. ✅ Prepare workspace and assembly tools
5. ✅ Begin hardware assembly on delivery day

## Useful Resources

**Platform Documentation:**
- [Waveshare Wave Rover Wiki](https://www.waveshare.com/wiki/WAVE_ROVER)
- [NVIDIA Jetson Orin Documentation](https://developer.nvidia.com/embedded/jetson-orin)
- [RPLidar C1 Manual](https://www.slamtec.com/en/Lidar/C1)

**ROS2 Learning:**
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

**Community:**
- [ROS Discourse Forum](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
- [r/ROS Subreddit](https://www.reddit.com/r/ROS/)

## Author

**Arsham Faghihnasiri**

Building autonomous systems and learning production ROS2 development.

- 📍 Location: Greater Toronto Area, Ontario, Canada
- 💼 LinkedIn: www.linkedin.com/in/arsham-faghihnasiri
- 📧 Email: arshamfaghihnasiri@gmail.com
- 🎓 Background: Software Engineering, Robotics

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

You are free to use, modify, and distribute this code for educational and commercial purposes.

## Acknowledgments

- Waveshare for the Wave Rover platform design
- NVIDIA for Jetson developer tools and documentation
- Slamtec for RPLidar SDK and ROS2 drivers
- Open Robotics and the ROS2 community
- Steve Macenski for slam_toolbox and Nav2 development

---

**Project Status:** 🟡 In Progress - Parts arriving February 19, 2025

**Built with:** NVIDIA Jetson Orin Nano Super | Waveshare Wave Rover | ROS2 Humble | Slamtec RPLidar C1

---

*"The best way to learn robotics is to build a robot."*
