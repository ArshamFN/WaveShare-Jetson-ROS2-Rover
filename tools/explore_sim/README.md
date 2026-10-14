# Exploration Simulator

An offline simulator that runs frontier exploration strategies on real saved maps of the
house and measures how far each one drives, how long it takes, and how much of the house
it sees. It was built in Session 016 to find out why `explore_lite` explored
inefficiently and to choose the design of `rover_explorer`. The `rover` policy runs
`src/rover_explore/rover_explore/frontier_planner.py`, the same file the robot runs, so
every decision the robot makes has been exercised here first.

## Model

- **Ground truth:** a saved slam_toolbox map from `maps/`. Free cells are free; occupied
  and never-seen cells are solid.
- **Lidar:** 360°, 12 m, 500 rays, like the RPLIDAR C1, with perfect returns. The
  simulated robot builds its own map from scratch.
- **Navigation:** modelled on the rover's Nav2 configuration: footprint 0.25 × 0.227 m
  (inscribed radius 0.1135 m), 0.22 m/s, a 0.6 rad/s spin in place when the heading
  error exceeds 0.5 rad, NavFn with unknown space allowed and a 0.25 m tolerance, a
  0.15 m goal tolerance, a 1 Hz replan while driving, and 20 s lost for every failed goal.
- **Starts:** the real start pose of each map (its map frame origin) plus random poses
  with 0.25 m of clearance in the largest free region, drawn with a fixed seed, so every
  policy runs from the same starts.
- **Limits:** perfect sensing and localization, and no Nav2 recovery behaviours beyond
  the time penalty. Real-world failures cost far more: on the Oct 7 map the simulated
  `explore_lite` needs about 160 s against 591 s on the robot. The simulator ranks
  designs; it does not predict run times.

## Policies

| Name | Policy |
|---|---|
| `A` | `explore_lite` at m-explore-ros2 `326cf8a` as configured on the rover: frontier cost `3.0 × straight-line distance − 1.0 × size`, goal at the frontier centroid, re-scored at 0.15 Hz, a frontier blacklisted when its goal aborts or makes no progress for 30 s |
| `A2` | `A` with `gain_scale` 0 and `planner_frequency` 0.05 |
| `B` | Sector rule: a frontier within 2 m, ±35° of the heading and in line of sight first, otherwise the nearest one |
| `C` | Path planner: reachability over known-free cells the footprint fits in, cost by driving distance plus a turn penalty, a viewpoint that sees the frontier, commitment to the current target, and giving up on a frontier only after reaching its viewpoint without learning anything |
| `D` | `C` plus a route through all frontiers, nearest neighbour improved with 2-opt on driving distances between viewpoints |
| `rover` | `rover_explorer`'s planner, `frontier_planner.py`, with its default parameters (the values in `src/rover_explore/config/explore_params.yaml`) |

## Requirements

Python 3 with numpy, scipy, scikit-image, and Pillow, on a desktop machine (scikit-image
is not installed on the Jetson):

```bash
pip install numpy scipy scikit-image pillow
```

The maps come from `maps/`: `s015_full_house_map` (map name `s015`) and
`explore_20261007-122528_final` (map name `oct7`).

## Usage

From this folder:

```bash
python3 compare.py s015 A rover                # 10 starts each; writes results/s015_A_n10.json and results/s015_rover_n10.json
python3 compare.py oct7 D --starts 30 --images runs   # 30 starts, plus one image per run in runs/
python3 summarize.py                           # comparison tables from results/
python3 figures.py                             # redraws the figures in images/testing/session-016/
./run_all.sh                                   # every comparison in the Session 016 log, about an hour on two cores
```

In the images, blue is the path driven, red the frontiers left unexplored at the end,
orange free space the robot never saw, and the green circle the start.

## Files

| File | Contents |
|---|---|
| `sim.py` | The simulator: map loading, lidar, belief map, frontier detection, and the navigation model |
| `policies.py` | Policies `A`, `A2`, `B`, `C`, and `D` |
| `module_policy.py` | Wraps `frontier_planner.py` as the `rover` policy; `ROVER_EXPLORE_PKG` overrides the package path |
| `compare.py` | Runs policies from shared starts and writes `results/` |
| `summarize.py` | Prints the comparison tables; starts where `explore_lite` hit the one-hour limit are left out for every policy on that map |
| `figures.py` | Draws the simulator figures in the Session 016 log |
| `viz.py` | Renders one finished run |
| `run_all.sh` | Re-runs every comparison and prints the tables |
| `results/` | Raw results of the runs in the Session 016 log, one JSON row per start |

## Results

Mean of 10 starts. On the Oct 7 map one `explore_lite` start oscillated between two
frontiers for the full simulated hour, so that start is left out for every policy.

| Map | `explore_lite` | `D` | `rover_explorer` | `rover_explorer` against `explore_lite` |
|---|---|---|---|---|
| s015 full house | 30.2 m | 22.8 m | 24.4 m | −19% |
| Oct 7, with boxes on the floor | 23.4 m | 17.8 m | 16.8 m | −28% |

`rover_explorer` keeps 0.25 m of clearance around its viewpoints whenever it can, which
costs some distance against `D` in simulation and avoids goals tucked against obstacles
on the robot. The full tables, the figures, and the reasoning behind each design choice
are in the
[Session 016 log](../../docs/testing/2026-10-06-session-016-Autonomous-Mapping-With-My-Own-Frontier-Explorer.md).
