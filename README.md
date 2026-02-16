# Autonomous Rover - Jetson + ROS2 Navigation

A 4WD autonomous ground robot built on the Waveshare Wave Rover platform with NVIDIA Jetson Orin Nano Super, implementing ROS2 Humble navigation and SLAM.

![Project Status](https://img.shields.io/badge/status-in%20progress-yellow)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/platform-Jetson%20Orin%20Nano-green)

## 📋 Project Overview

**Goal:** Build a complete autonomous navigation system from scratch to demonstrate production-ready ROS2 development skills for robotics engineering roles.

**Why This Project:**
- Learn ROS2 development (nodes, topics, services, actions)
- Implement SLAM-based mapping and localization
- Configure autonomous waypoint navigation with Nav2
- Document complete build process for portfolio
- Future-ready for computer vision integration

## 🤖 Hardware Platform

### Core Components
- **Platform:** Waveshare Wave Rover (4WD skid-steer chassis)
- **Compute:** NVIDIA Jetson Orin Nano Super Developer Kit (8GB RAM, 67 TOPS AI)
- **Sensors:** Slamtec RPLidar C1 (12m range, 10Hz, DTOF)
- **Power:** 3x 18650 lithium batteries (7800mAh) via Wave Rover UPS module
- **Communication:** Onboard ESP32 for low-level motor control

### Technical Specifications
| Component | Specs |
|-----------|-------|
| **Drivetrain** | 4x N20 motors with encoders, skid-steer kinematics |
| **CPU** | 6-core ARM Cortex-A78AE @ 1.7GHz |
| **GPU** | 512 CUDA cores + Tensor cores |
| **Lidar Range** | 12m (white objects), 6m (black objects) |
| **Scan Rate** | 5000 Hz @ 10Hz rotation |
| **Weight** | ~2.5kg total |

[📸 Hardware photos coming soon]

## 💻 Software Stack

- **Operating System:** Ubuntu 22.04 (JetPack 6.1)
- **ROS Version:** ROS2 Humble Hawksbill
- **Navigation:** Nav2 (Navigation2 stack)
- **SLAM:** slam_toolbox
- **Localization:** AMCL (Adaptive Monte Carlo Localization)
- **Visualization:** RViz2
- **Languages:** Python 3, C++

## 📅 Project Timeline

### Phase 1: Hardware Setup & Integration (Weeks 1-2)
- [x] Parts ordered
- [ ] Unbox and inventory all components
- [ ] Assemble Wave Rover base
- [ ] Mount Jetson on rover
- [ ] Mount RPLidar C1
- [ ] Test power system
- [ ] Verify motor control via ESP32

**Milestone:** Complete mechanical assembly, all systems powered

### Phase 2: ROS2 Base Setup (Weeks 3-4)
- [ ] Install JetPack 6.1 on Jetson
- [ ] Install ROS2 Humble
- [ ] Configure ESP32-ROS2 serial bridge
- [ ] Create motor control node
- [ ] Test teleoperation (keyboard control)
- [ ] Verify encoder odometry

**Milestone:** Drive rover via ROS2 commands

### Phase 3: Sensor Integration (Weeks 5-6)
- [ ] Install RPLidar ROS2 driver
- [ ] Visualize scan data in RViz2
- [ ] Configure TF tree (transforms)
- [ ] Integrate wheel odometry
- [ ] Test sensor fusion

**Milestone:** Live sensor visualization in RViz2

### Phase 4: SLAM Mapping (Weeks 7-8)
- [ ] Install slam_toolbox
- [ ] Configure SLAM parameters for RPLidar C1
- [ ] Create map of test environment
- [ ] Tune loop closure detection
- [ ] Save and load maps

**Milestone:** Reliable map generation

### Phase 5: Autonomous Navigation (Weeks 9-11)
- [ ] Install Nav2 stack
- [ ] Configure costmaps (global & local)
- [ ] Set up path planning (NavFn, Smac Planner)
- [ ] Configure controller (DWB, TEB)
- [ ] Test waypoint navigation
- [ ] Tune navigation parameters

**Milestone:** Autonomous waypoint-to-waypoint navigation

### Phase 6: Testing & Documentation (Weeks 12-14)
- [ ] Run full autonomous missions
- [ ] Record demo videos
- [ ] Complete all documentation
- [ ] Write technical blog post
- [ ] Prepare for job applications

**Milestone:** Portfolio-ready project

## 📂 Project Structure
```
waveshare-jetson-ros2-rover/
├── README.md
├── docs/
│   ├── hardware/          # Assembly guides, wiring diagrams, BOM
│   ├── software/          # Setup instructions, architecture docs
│   └── testing/           # Test logs, issues encountered, solutions
├── src/                   # ROS2 packages
│   ├── rover_bringup/     # Launch files, configurations
│   ├── rover_description/ # URDF models
│   └── rover_navigation/  # Navigation configurations
├── images/                # Photos and diagrams
└── cad/                   # 3D models for custom mounts
```

## 🎯 Skills Demonstrated

This project showcases:
- ✅ **ROS2 Architecture:** Nodes, topics, services, actions, parameters
- ✅ **Mobile Robot Navigation:** Differential/skid-steer kinematics, odometry
- ✅ **SLAM:** Mapping, localization, loop closure
- ✅ **Sensor Integration:** Lidar, encoders, IMU
- ✅ **System Integration:** Hardware + software debugging
- ✅ **Technical Documentation:** Clear communication of complex systems
- ✅ **Problem Solving:** Real-world troubleshooting and optimization

## 🚀 Future Enhancements (Project #2)

After completing autonomous navigation:
- [ ] Add depth camera (Intel RealSense D435i)
- [ ] Implement visual obstacle detection (YOLO, OpenCV)
- [ ] Add GPS for outdoor waypoint navigation
- [ ] Multi-robot coordination
- [ ] Payload delivery mechanism

## 📚 Documentation

- [Bill of Materials](docs/hardware/bill-of-materials.md)
- [Assembly Guide](docs/hardware/assembly-guide.md)
- [Wiring Diagram](docs/hardware/wiring-diagram.md)
- [Software Setup](docs/software/setup-guide.md)
- [ROS2 Architecture](docs/software/architecture.md)
- [Test Logs](docs/testing/test-logs.md)

## 🎥 Demo Videos

*Coming soon - autonomous navigation demos*

## 📊 Current Status

**Last Updated:** [Today's date]

**Current Phase:** Phase 1 - Parts ordering and planning

**Next Steps:**
1. Order all components
2. Set up development environment
3. Begin hardware assembly when parts arrive

## 🔗 Resources

- [Waveshare Wave Rover Wiki](https://www.waveshare.com/wiki/WAVE_ROVER)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## 👤 Author

**[Your Name]**
- LinkedIn: [Your Profile]
- Email: [Your Email]
- Location: GTA, Ontario, Canada

## 📄 License

MIT License - See LICENSE file for details

---

**Built with:** NVIDIA Jetson Orin Nano Super | Waveshare Wave Rover | ROS2 Humble | RPLidar C1
