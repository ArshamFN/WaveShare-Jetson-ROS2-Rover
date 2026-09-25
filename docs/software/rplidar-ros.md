# rplidar_ros

ROS2 driver for the Slamtec RPLidar series, including the RPLidar C1 used in this project.
This package is sourced directly from Slamtec's official repository and included here for
completeness and reproducibility.

**Original repository:** https://github.com/Slamtec/rplidar_ros  
**Branch used:** `ros2`  
**License:** See `LICENSE` file in this directory.

---

## Hardware

This setup uses the **Slamtec RPLidar C1** — a low-cost DTOF (Direct Time-of-Flight) LiDAR
with 360° scanning, up to 12m range, and a 10Hz scan rate. It connects to the Jetson via
USB through an onboard CP2102 adapter.

---

## Prerequisites

- ROS2 Humble installed and sourced
- RPLidar C1 connected via USB
- `/dev/lidar` udev symlink configured (see udev setup below)

---

## udev Rule Setup

A udev rule gives the LiDAR a persistent device name that never changes between reboots.

**Step 1 — Find the device's hardware identifiers.**

With the LiDAR plugged in, run:

```bash
udevadm info -a -n /dev/ttyUSB* | grep -E "idVendor|idProduct|serial" | head -10
```

Look for the block with `idVendor=="10c4"` (Silicon Labs CP2102). Note the `serial` value.

**Step 2 — Create the udev rule.**

```bash
sudo nano /etc/udev/rules.d/99-rover.rules
```

Add the following line, replacing the serial value with yours:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="YOUR_SERIAL_HERE", SYMLINK+="lidar", MODE="0666"
```

If you also have a `/dev/rover` rule for the GRD, both rules go in the same file — one line each.

**Step 3 — Apply the rule.**

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Step 4 — Verify.**

```bash
ls -la /dev/lidar
```

Expected output:
```
lrwxrwxrwx 1 root root 7 ... /dev/lidar -> ttyUSB0
```

---

## Why Build From Source

The version of `rplidar_ros` available via `apt` does not include support for the C1 model —
the C1 is a newer DTOF device added to the repository after the apt package was released.
Building from source (specifically the `ros2` branch of Slamtec's official repository)
is required to get full C1 support including the correct launch file.

---

## Workspace Setup

**Step 1 — Create a ROS2 workspace if you don't have one.**

```bash
mkdir -p ~/ros2_ws/src
```

**Step 2 — Copy this package into your workspace.**

```bash
cp -r rplidar_ros ~/ros2_ws/src/
```

Alternatively, clone directly from Slamtec and check out the ros2 branch:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd rplidar_ros
git checkout ros2
```

**Step 3 — Build the package.**

```bash
cd ~/ros2_ws
colcon build --packages-select rplidar_ros
```

Expected output:
```
Starting >>> rplidar_ros
Finished <<< rplidar_ros [~35s]
Summary: 1 package finished
```

The build produces a large number of compiler warnings from Slamtec's SDK — these are
normal and do not affect functionality. The build is successful as long as it shows
`Finished` with no errors.

**Step 4 — Source the workspace.**

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Running the LiDAR Driver

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py
```

Expected output:
```
[rplidar_node-1] [INFO] [rplidar_node]: RPLidar running on ROS2 package rplidar_ros. RPLIDAR SDK Version:2.1.0
[rplidar_node-1] [INFO] [rplidar_node]: RPLidar health status : OK.
[rplidar_node-1] [INFO] [rplidar_node]: current scan mode: Standard, sample rate: 5 Khz, max_distance: 16.0 m, scan frequency:10.0 Hz,
```

The LiDAR motor will spin up and the node will begin publishing scan data.

---

## Verifying Scan Data

With the driver running in one terminal, open a second terminal and subscribe to the
`/scan` topic:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /scan --once
```

A successful result is a large block of distance measurements covering 360° around the
sensor. This is the raw data that will feed into the SLAM algorithm to generate maps.

---

## Troubleshooting

**Launch file not found:**
Ensure you built from the `ros2` branch, not the default `master` branch. The `master`
branch is the ROS1 version and uses a different build system entirely.

```bash
cd ~/ros2_ws/src/rplidar_ros
git branch
```

Should show `* ros2`. If it shows `* master`, run:

```bash
git checkout ros2
cd ~/ros2_ws
colcon build --packages-select rplidar_ros
```

**No scan data / health status error:**
Check that `/dev/lidar` exists and the LiDAR is properly powered via USB:

```bash
ls -la /dev/lidar
```

The LiDAR draws approximately 2.5W from the USB port — ensure the Jetson's USB port
can supply sufficient current.
