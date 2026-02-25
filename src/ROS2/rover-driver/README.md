# rover_driver

A ROS2 node that bridges the standard `/cmd_vel` topic to the WaveShare Wave Rover's
JSON-over-serial protocol. This is the translation layer between ROS2's universal movement
interface and the rover's ESP32-based General Driver Board.

---

## How It Works

ROS2 uses a standard message type called `geometry_msgs/Twist` to represent robot velocity.
Every navigation tool, teleoperation package, and autonomous planner in the ROS2 ecosystem
publishes to `/cmd_vel` using this format — it specifies `linear.x` (forward/backward speed)
and `angular.z` (rotation speed).

The Wave Rover's General Driver Board speaks a completely different language: JSON commands
sent over a USB serial connection, where `{"T":1,"L":0.5,"R":0.5}` means "drive both motors
at 50% forward."

This node sits between the two. It subscribes to `/cmd_vel`, converts linear and angular
velocity into differential left/right wheel speeds using standard unicycle drive kinematics,
and writes the corresponding JSON command to `/dev/rover` over USB serial.

The result: any ROS2 tool that publishes to `/cmd_vel` — whether that's a keyboard
teleoperation node, a joystick controller, or a full Nav2 autonomous navigation stack —
will drive the rover without any modification.

---

## Prerequisites

Before setting up this package, ensure the following are in place:

**Hardware:**
- WaveShare Wave Rover with General Driver Board (ESP32)
- NVIDIA Jetson Orin Nano Super (or compatible host)
- USB-C cable connecting the GRD's USB port to the Jetson

**Software:**
- ROS2 Humble installed and sourced
- `pyserial` available (`pip3 install pyserial --break-system-packages`)
- `/dev/rover` udev symlink configured (see udev setup below)

---

## udev Rule Setup

A udev rule assigns a persistent symlink to the GRD's USB connection so the port name
never changes between reboots or reconnections.

**Step 1 — Find the device's hardware identifiers.**

Plug in the GRD USB cable and run:

```bash
udevadm info -a -n /dev/ttyUSB* | grep -E "idVendor|idProduct|serial" | head -10
```

Look for the block with `idVendor=="10c4"` (Silicon Labs CP2102). Note the `serial` value —
it's a long hex string unique to your specific GRD unit.

**Step 2 — Create the udev rule.**

```bash
sudo nano /etc/udev/rules.d/99-rover.rules
```

Add the following line, replacing the serial value with yours:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="YOUR_SERIAL_HERE", SYMLINK+="rover", MODE="0666"
```

**Step 3 — Apply the rule.**

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Step 4 — Verify.**

```bash
ls -la /dev/rover
```

Expected output:
```
lrwxrwxrwx 1 root root 7 ... /dev/rover -> ttyUSB1
```

---

## Workspace Setup

**Step 1 — Create a ROS2 workspace if you don't have one.**

```bash
mkdir -p ~/ros2_ws/src
```

**Step 2 — Copy this package into your workspace.**

```bash
cp -r rover_driver ~/ros2_ws/src/
```

**Step 3 — Build the package.**

```bash
cd ~/ros2_ws
colcon build --packages-select rover_driver
```

Expected output:
```
Starting >>> rover_driver
Finished <<< rover_driver [~2s]
Summary: 1 package finished
```

**Step 4 — Source the workspace.**

```bash
source ~/ros2_ws/install/setup.bash
```

To make this permanent so every new terminal has the package available:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Running the Node

```bash
ros2 run rover_driver rover_driver_node
```

Expected output:
```
[INFO] [rover_driver]: Rover driver node started, listening on /cmd_vel
```

The node is now active and waiting for movement commands. Stop it at any time with
`Ctrl+C` — it will automatically send a stop command to the rover before shutting down.

---

## Testing Motor Control

With the node running in one terminal, open a second terminal and publish a test command:

```bash
# Move forward at 50% speed
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

The node terminal will log each command received and the resulting wheel speeds:
```
[INFO] [rover_driver]: CMD: linear=0.50 angular=0.00 -> L=0.500 R=0.500
```

---

## Velocity Conversion

The node uses standard unicycle drive kinematics to convert `Twist` velocity into
differential wheel speeds:

```
left  = linear.x - angular.z × 0.5
right = linear.x + angular.z × 0.5
```

Both values are clamped to `[-1.0, 1.0]` before being sent to the GRD, where `1.0`
represents 100% PWM forward and `-1.0` represents 100% PWM reverse.

---

## Troubleshooting

**Node fails to start with serial port error:**
Verify `/dev/rover` exists and the udev rule is applied:
```bash
ls -la /dev/rover
```

**Commands received but rover doesn't move:**
Check that the GRD is powered on (OLED should be active) and the USB cable is fully seated.
Verify communication directly:
```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/rover', 115200, timeout=2)
s.write(b'{\"T\":10031}\n')
time.sleep(0.5)
print(s.read(200))
s.close()
"
```
Expected response: `b'{"T":10031}\n'`
