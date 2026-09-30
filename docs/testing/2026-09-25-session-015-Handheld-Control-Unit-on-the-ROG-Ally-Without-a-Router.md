# Session 015 — 2026-09-25: Handheld Control Unit on the ROG Ally Without a Router

**Date:** 2026-09-25  
**Status:** ✅ Complete

---

## Goal

Operate the rover from a handheld device anywhere in the house. I wanted the live map,
the scan and the robot pose, Nav2 goals, stick driving behind a deadman, starting and
stopping the stack, saving maps, and a safe shutdown, all from my ASUS ROG Ally, with no
laptop, no NoMachine session, and no dependence on the home router.

---

## Context

After Session 014 the rover navigated reliably, but operating it still meant sitting at
a desk with a NoMachine session open on the Jetson. The session ran five days, from
2026-09-25 to 2026-09-29.

The Ally stays on Windows 11. Installing Ubuntu 22.04 on it risked losing the built-in
gamepad and Wi-Fi, and with a single USB-C port and a broken SD card reader, a dual boot
was impractical. Whatever ran on the Ally had to be a Windows program.

My requirements going in:

- battery voltage on screen;
- start and stop for bringup and Nav2;
- switching between Nav2 and stick driving;
- a live map view with the scan, the robot position, goal selection, and Nav2's planned
  path;
- saving the map on the Jetson under a name I type.

Two more were added during the session: a direct connection to the rover with no router
in between, and a safe shutdown from the Ally.

The original plan was a custom Windows app: PySide6, packaged as an `.exe`, talking to
rosbridge, controlling services over SSH, and drawing the map itself.

---

## Discovery 1 — The Pendant Needed Robot-Side Plumbing, Not a Custom App

Listing what the app would have to draw changed the plan before I wrote any of it.
Rendering maps, scans, TF, costmaps and paths, and publishing goals, is exactly what
Foxglove already does. I dropped the custom app: Foxglove desktop runs on Windows,
`foxglove_bridge` runs on the Jetson, and the custom work moved entirely to the robot
side. A custom client is still possible later, specified from the gaps I find in real
use rather than guessed in advance.

Before installing anything I did a read-only audit of the Jetson. `foxglove_bridge`
3.5.0 and `twist_mux` 4.3.0 were available from the ROS apt repository, and
`teleop_twist_joy` 2.4.8 was already installed. The Wi-Fi card (Realtek, driver
`rtl88x2ce`, interface `wlP1p1s0`) supports access point mode on 2.4 and 5 GHz, but it
reports no interface combinations, so it cannot host a network and join one at the same
time, and its regulatory country is unset (`00`). A user systemd instance was running
through GDM auto-login, with lingering off. All three of those facts came back later.

Foxglove desktop opened and accepted a connection address with the Ally offline, with
no sign-in wall. It needs one sign-in while online, and after that it works offline.

Over home Wi-Fi, `foxglove_bridge` on port 8765 carried the live map, scan, TF and robot
pose into the 3D panel, along with the costmaps, Nav2's plan, a gauge on
`/battery_voltage`, and the Ally's controls through a joystick panel extension that
publishes `sensor_msgs/Joy` on `/joy`.

Compute was not a problem. At the 25 W power mode, with bringup, `foxglove_bridge`, Nav2
and joystick teleop running and Foxglove streaming, RAM peaked at about 3.1 of 7.4 GB,
the busiest single core reached 78 percent, and the highest all-core average was 70
percent.

Two settings failed silently, and each cost a round of debugging:

- The joystick panel's Publish Mode was off, so `/joy` had a subscriber and no
  publisher.
- The 3D panel's pose publish topic defaulted to `/move_base_simple/goal`, the ROS 1
  name, and when I changed it I typed `goalpose` instead of `/goal_pose`.

Neither produced an error anywhere. Both showed up only when I checked the robot's side
with `ros2 topic info` and `ros2 topic list`.

Seeing the costmaps on the Ally, Nav2's inflation looked oversized for narrow gaps. A
live parameter dump showed the measured footprint in both costmaps, so I changed
nothing.

---

## Discovery 2 — Nav2 Publishes Velocity From Two Places, So the Mux Sits on the Driver's Input

Stick driving and Nav2 both need to reach the wheels, with the stick winning. The
obvious design is to remap Nav2's output into a mux. Reading the installed Humble launch
file showed that Nav2 has two outputs, not one: `controller_server` publishes
`/cmd_vel_nav`, the velocity smoother turns that into `/cmd_vel`, and the behavior server
(spin, backup, wait) publishes `/cmd_vel` directly. Both remappings live in the
system-installed file, which I did not want to fork.

So Nav2 stays untouched. `/cmd_vel` became one input of `twist_mux`, and the only edit
to the stack was remapping `rover_driver`'s input to `/cmd_vel_mux` in
`bringup.launch.py`. The mux has two inputs and two locks:

| Input or lock | Topic | Priority | Timeout |
|---|---|---|---|
| Navigation | `/cmd_vel` | 10 | 0.5 s |
| Joystick | `/cmd_vel_joy` | 100 | 0.5 s |
| Teleop-mode lock | `/pendant/teleop_mode` | 50 | none |
| E-stop lock | `/pendant/estop` | 255 | none |

`teleop_twist_joy` runs inside bringup with no `joy_node`, because `/joy` comes from the
pendant. The deadman is LB (button 4), the left stick's axis 1 drives linear speed up to
0.25 m/s, and axis 0 drives angular speed up to 1.5 rad/s. The signs came out right
without negation, and 0.25 m/s stays within what SLAM tolerates without smearing the
map.

Before adding the mux I tested the stick on a stand with the wheels off the ground.
Releasing LB stopped the wheels, and so did switching the Ally to another app while
holding LB and the stick.

With the mux in place I ran eight checks, still on the stand:

1. `/cmd_vel_mux` had one publisher and one subscriber.
2. The stick drove the wheels.
3. A Nav2 goal drove the wheels.
4. Holding LB during a goal took over, and Nav2 resumed on release.
5. The teleop-mode lock stopped Nav2 while the stick still worked.
6. The e-stop lock stopped everything, the stick included.
7. Releasing each lock restored driving.
8. Switching apps while driving stopped the wheels.

All eight passed. The design has limits I accept for now. The locks fail open, so a
pendant that goes silent does not trigger the e-stop, and the e-stop itself travels over
Wi-Fi. It is a convenience stop; the chassis power switch is the real emergency stop.
Keyboard teleop now enters on the navigation input, so either lock blocks it.

---

## Discovery 3 — Stopping the Stack Has to Stop the Wheels

Starting and stopping the stack from the pendant meant systemd services instead of
terminals. The risk was in the stop. The MFD board keeps executing its last wheel
command, so killing the stack mid-drive could leave the wheels turning with nothing left
to stop them.

I wrote four systemd user services. `rover-bridge` and `rover-control` start at boot,
which needed lingering enabled, since it was off. `rover-bringup` and `rover-nav2` start
on demand, and Nav2 stops whenever bringup stops. After every stop or crash,
`rover-bringup` runs `rover_zero_motors.py`, which sends the zero wheel command twice,
50 ms apart. The test was the case I was worried about: stopping bringup from the
pendant while the wheels were turning stopped them, and Nav2 stopped with it.

The services are driven by a new node, `pendant_control`. It publishes both services'
state at 1 Hz and a one-line result for every command, and it starts and stops each
service. It saves the live map by name: letters, digits, underscore and hyphen, up to 64
characters, only while bringup runs, never overwriting an existing file, with
`map_saver_cli` given 30 s. It also refuses to start bringup or Nav2 while a copy started
outside systemd is running, so the pendant and a desktop launcher cannot start a second
stack on top of the first.

The first version had separate start and stop services. I replaced them on the pendant
with one toggle button per service. Each toggle reads the live systemd state, refuses
while the service is starting or stopping, and ignores presses less than 2 s apart.

Safe shutdown takes two presses within 5 s. It stops Nav2, then bringup, then powers off
through a sudo rule that allows only `/usr/bin/systemctl poweroff`, which I validated
with `visudo` before installing it. Poweroff happens only when the node's
`allow_poweroff` parameter is true, and only the installed service sets it, so running
the node by hand stops the stack but leaves the Jetson on.

On hardware, the toggles, the status indicators, saving a map, and the two-press
shutdown all worked. A duplicate map name and `../bad` were both refused. After a reboot
with no hands on the Jetson, Foxglove reconnected and the pendant started bringup. The
first full-house map from the pendant is `maps/s015_full_house_map`.

One reboot looked like the pendant had broken: the buttons did nothing. My first
suspicion was boot order. The control node's log showed otherwise. Every press was
arriving as a stop request, because the BRINGUP panel still held `{"data": false}` from
the stop test. The fault was in the panel, not on the Jetson.

A practical note: this Jetson keeps its journal in memory only, and user-service logs
are read with `journalctl --user-unit <unit>`.

---

## Discovery 4 — The Jetson Can Host Its Own Network, but Only on 2.4 GHz

Everything so far depended on the home router. I expected the Ally to be the easy side
of a direct link, since Windows has a Mobile Hotspot. It does not work here: Mobile
Hotspot needs an upstream internet connection before it will start.

I considered the other radios before settling on Wi-Fi. The pendant's load is roughly
1 Mbps, estimated from message sizes and rates, and it grows with the map. That is at or
beyond what Bluetooth networking delivers in practice, with roughly 10 m of indoor range
and clumsy network support on Windows. LoRa, Zigbee, 915 MHz telemetry radios and nRF24
are far too slow. Wi-Fi HaLow is fast and long range, but its Windows support is
immature, and its range matters outdoors, not in a house.

So the Jetson hosts the network. It runs a NetworkManager hotspot: profile `rover-ap`,
SSID `NullIsland-Rover`, WPA2, with the Jetson at `10.42.0.1` handing out addresses.

My first attempt was 5 GHz, channel 36. `wpa_supplicant` reported
`Failed to start AP functionality` immediately, and NetworkManager gave up about 25 s
later. That is consistent with the unset regulatory country from the audit. On 2.4 GHz,
channel 6, the hotspot came up at once.

Because the card cannot host and join at the same time, starting the hotspot takes the
Jetson off home Wi-Fi, and a failure could leave it unreachable. Every hotspot test ran
behind a timer (`systemd-run --on-active`) that rejoined home Wi-Fi after 5 to 15
minutes regardless of state. One failed attempt did leave the Jetson with no Wi-Fi
connection at all, and the timer brought it back.

Over the hotspot the Ally got `10.42.0.31`, and Foxglove connected and streamed with no
internet anywhere. One connection attempt on 2026-09-28 failed with no error detail and
was never explained; the attempts after it connected.

Connecting was only the first half. Using the pendant over the hotspot led directly to
the next discovery.

---

## Discovery 5 — Switching Networks Split the ROS Graph

Over the hotspot the buttons and indicators worked, but the map, the battery gauge and
the joystick did not. The order of events mattered: `foxglove_bridge` and the control
node had started at boot on home Wi-Fi, and bringup started after the switch to the
hotspot.

ROS 2 discovery advertises the addresses a node had when it started. The nodes that
started before the switch kept advertising the home address, so they could not find the
nodes that started after it. Pairs that had already found each other before the switch
kept working, which is why the buttons still did: the bridge and the control node had
met on home Wi-Fi.

The fix came from noticing that discovery never needed Wi-Fi in the first place. Every
ROS node runs on the Jetson, and only Foxglove crosses the network, over its own
WebSocket. So `ROS_LOCALHOST_ONLY=1` is now exported in all four services (after the ROS
setup files are sourced), in the three desktop launchers, and in `~/.bashrc`.

I verified it in three steps: on the running processes through `/proc/<pid>/environ`,
then at home (bringup, stick, a Nav2 goal), then over the hotspot with bringup started
after the switch. Everything worked with no restarts.

The motor board and the lidar are USB serial devices and are unaffected. RViz still
works, as long as it runs on the Jetson itself, through NoMachine.

---

## Discovery 6 — Leaving the Hotspot Safely Depends on Knowing the Pendant Is Connected

With the graph no longer tied to an address, the Jetson could change networks freely,
so I let NetworkManager handle boot. `rover-ap` autoconnects at priority -100, below
every other profile (the home network is at 10), so home wins whenever it is in range.
Six reboots with no hands on the Jetson: 3 of 3 joined home Wi-Fi with the router on,
and 3 of 3 started the hotspot with it unplugged.

That fallback raised the opposite problem. Without anything more, the Jetson would stay
on the hotspot until the next reboot, even after home Wi-Fi came back. The simple fix,
switching back as soon as home is visible, would cut the pendant off mid-run while Nav2
was still driving the rover.

`rover-net-watchdog` is a root system service running a root-owned copy of the script in
`/usr/local/sbin`. It checks every 15 s:

- No Wi-Fi connection for 60 s: it starts the hotspot.
- Hotspot with a client connected: it takes no action, however long.
- Hotspot with no client for 5 minutes: it tries home Wi-Fi for up to 40 s, and
  restores the hotspot if that fails.

Its client sensor is `iw dev wlP1p1s0 station dump`. Read on the hardware, it listed 1
station with the Ally connected, at -32 dBm, and 0 after it left. In client mode it
lists nothing.

The first version had three ways to fail unsafe, all fixed before it was committed. A
client list that could not be read counted as zero clients, so after 5 minutes it would
have dropped a connected Ally; I noticed this while writing the deployment audit. The
audit's review found the other two. A NetworkManager read that failed counted as "no
connection", which could move the rover off home Wi-Fi or restart the hotspot under the
Ally. And no call had a time limit, so a hung command would freeze the loop without a
log line, and `Restart=always` would never fire, because the process never exits.

The rule now is that a reading that fails or hangs causes no action. There is one read
per pass; every external call is time-limited (reads 10 s, hotspot start 30 s, home
attempt 40 s, each with a 10 s backstop); and the reason for any failure is written to
the log. A client list that cannot be read counts as a client connected.

Verification came in layers. The 22-check deployment audit (checksums of the repository
and installed copies, permissions, unit state, the running process, the network
profiles) passed on the previous version. The final version was confirmed by checksum
against the repository and a clean start. `shellcheck` is clean. The loop also ran
against stand-in `nmcli` and `iw` commands in 14 scenarios, including failing and
hanging calls, and it never dropped a connected client or acted on an unknown state.

What was not tested on the rover: losing home Wi-Fi while running, the idle return to
home, and a failed home attempt. I chose to skip that test for now.

The trade-off is deliberate. In the field with nobody connected, the hotspot disappears
for up to about 70 s every 5 minutes while the rover looks for home.

---

## Solution / What Was Done

### Velocity chain

`src/robot_description/config/twist_mux.yaml`:

```yaml
twist_mux:
  ros__parameters:
    topics:
      navigation:
        topic: /cmd_vel
        timeout: 0.5
        priority: 10
      joystick:
        topic: /cmd_vel_joy
        timeout: 0.5
        priority: 100
    locks:
      teleop_mode:
        topic: /pendant/teleop_mode
        timeout: 0.0
        priority: 50
      estop:
        topic: /pendant/estop
        timeout: 0.0
        priority: 255
```

The one change to the driver's wiring, in `bringup.launch.py`:

```python
remappings=[('/cmd_vel', '/cmd_vel_mux')],
```

`twist_mux` publishes on `/cmd_vel_mux`, and `teleop_twist_joy` publishes on
`/cmd_vel_joy`, with the deadman, axes and scales in `config/teleop_joy.yaml`
(`enable_button: 4`, linear axis 1 at 0.25, angular axis 0 at 1.5).

### Services and the zero command

| Service | Starts | Runs |
|---|---|---|
| `rover-bridge` | at boot | `foxglove_bridge` |
| `rover-control` | at boot | `pendant_control`, with `allow_poweroff:=true` |
| `rover-bringup` | on demand | `bringup.launch.py use_rviz:=false` |
| `rover-nav2` | on demand | Nav2, `PartOf=rover-bringup.service` |

The zero command runs after every stop or crash of bringup:

```ini
ExecStopPost=/usr/bin/python3 %h/ros2_ws/tools/systemd/rover_zero_motors.py
```

It opens `/dev/rover` and writes `{"T":1,"L":0,"R":0}` twice, 50 ms apart.
`tools/systemd/install.sh` installs the units and enables the two boot services, and
warns if lingering is off.

### Pendant control node

`pendant_control` in the new `pendant_bridge` package offers `/pendant/toggle_bringup`,
`/pendant/toggle_nav2`, `/pendant/set_bringup`, `/pendant/set_nav2`,
`/pendant/save_map` and `/pendant/shutdown`, and publishes `/pendant/bringup_active`,
`/pendant/nav2_active` and `/pendant/control_result`. The sudo rule behind the shutdown,
installed as `/etc/sudoers.d/rover-poweroff`:

```
<user> ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff
```

The node calls it as `sudo -n /usr/bin/systemctl poweroff`, so a missing rule fails
immediately instead of waiting for a password.

### The rover's network

The hotspot profile, created with autoconnect off and turned on only after the password
was set (typed with `read -rs`, never saved in shell history):

```bash
nmcli con add type wifi ifname wlP1p1s0 con-name rover-ap ssid NullIsland-Rover \
  802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
  ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled \
  wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
  connection.autoconnect no connection.autoconnect-priority -100
```

The home profile's `connection.autoconnect-priority` is 10, so it always wins at boot
when it is in range.

### Localhost discovery

In each service the export sits inside the `ExecStart` command, after both setup files
are sourced and before the `exec`:

```ini
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source %h/ros2_ws/install/setup.bash && export ROS_LOCALHOST_ONLY=1 && exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
```

The three launcher scripts export it after `source install/setup.bash`, and `~/.bashrc`
carries `export ROS_LOCALHOST_ONLY=1` after the ROS setup lines.

### Network watchdog

`tools/network/rover-net-watchdog.sh` is installed as `/usr/local/sbin/rover-net-watchdog`
and run by `rover-net-watchdog.service` with `Restart=always`. The interface and
connection names are set at the top of the script, with these constants:

| Constant | Value | Role |
|---|---|---|
| `CHECK_EVERY` | 15 s | Time between checks |
| `NO_LINK_BEFORE_AP` | 60 s | No Wi-Fi connection this long starts the hotspot |
| `IDLE_BEFORE_HOME` | 300 s | Hotspot with no client this long tries home Wi-Fi |
| `HOME_TRY_TIMEOUT` | 40 s | Limit for one home Wi-Fi attempt |
| `AP_START_TIMEOUT` | 30 s | Limit for one hotspot start |
| `READ_TIMEOUT` | 10 s | Limit for any status read |

The two `nmcli --wait` calls also run under `timeout` with 10 s more than their own
limit, as the backstop.

---

## Result

The Ally is now a handheld control unit for the rover. From anywhere in the house, with
or without the router, I can see the live map, scan, robot pose, costmaps and Nav2's
plan; send Nav2 goals; drive with the stick behind LB, overriding Nav2; start and stop
bringup and Nav2 with one button each; save the map under a typed name; watch the
battery; and shut the Jetson down with two presses. The Jetson joins home Wi-Fi at boot
when it can and hosts `NullIsland-Rover` when it cannot, and a Wi-Fi change no longer
splits the ROS graph.

| Component | Publishes | Owns TF |
|---|---|---|
| `rplidar_ros` | `/scan` @ 10 Hz | none |
| `rf2o_laser_odometry` | `/odom` @ 10 Hz | **`odom → base_link`** |
| `rover_driver` | `/odom_wheel`, `/imu/gz`, `/battery_voltage` @ ~12 Hz; input `/cmd_vel_mux` | none |
| `slam_toolbox` | `/map` | `map → odom` |
| `robot_state_publisher` | none | `base_link → laser` |
| `twist_mux` | `/cmd_vel_mux` | none |
| `teleop_twist_joy` | `/cmd_vel_joy` | none |
| `foxglove_bridge` | WebSocket, port 8765 | none |
| `pendant_control` | `/pendant/bringup_active`, `/pendant/nav2_active`, `/pendant/control_result` | none |

Open items:

- The watchdog's running cycle is untested on the rover.
- With nobody connected in the field, the hotspot drops for up to about 70 s every 5
  minutes.
- There are no pendant buttons yet for the e-stop and teleop-mode locks.
- The locks fail open, and the e-stop travels over Wi-Fi.
- The Save Map panel needs its editing mode to type a name.
- Battery voltage is not visible until bringup starts.
- One Foxglove connection failure over the hotspot is still unexplained.
- The hotspot runs on 2.4 GHz only.
- The Foxglove layout is not in the repository.
- Some setup lives outside the repository: the `~/.bashrc` line, the sudo rule, the
  hotspot profile, the network priorities, and the watchdog install. The README lists
  them.

[![The handheld control unit driving the rover while Nav2 navigates the house](https://img.youtube.com/vi/RMOsRye9tLc/maxresdefault.jpg)](https://www.youtube.com/watch?v=RMOsRye9tLc)

*[Watch on YouTube](https://www.youtube.com/watch?v=RMOsRye9tLc): the handheld control unit driving the rover while Nav2 navigates the house.*

---

## Lessons Learned

**Unknown is not zero.** Twice in one script, a failed reading looked exactly like a
real one, and either would have cut the pendant off mid-run. It is the same class as
Session 014's lesson about logged errors that execution continues past.

**On one computer, ROS 2 discovery does not need the network.** The graph split came
from advertising Wi-Fi addresses that were never needed, since every node runs on the
Jetson and only Foxglove crosses the network.

**Build the way back before changing the network.** When a failed hotspot attempt left
the Jetson with no connection, the timer rejoined home Wi-Fi on its own, so no mistake
could strand the rover off the network.

**Check what the system received before theorizing.** Publish Mode, `goalpose`, and a
panel still sending `false` all looked right on screen. The robot's side
(`ros2 topic info`, `ros2 topic list`, the control node's log) showed the truth each
time.

---

## Next Session Goal

**Autonomous mapping.** The rover drives around the house on its own, builds the map as
it goes, and saves it when the map is complete.
