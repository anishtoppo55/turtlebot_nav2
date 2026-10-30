# TurtleBot3 Adaptive Path Replanner

A TurtleBot3 (burger) simulation on ROS 2 Humble with Gazebo Classic and Nav2. It adds:

- **`adaptive_path_replanner`**: a node that detects when the robot is stuck and raises the global costmap inflation radius, so Nav2 looks for a different route.
- **Obstacle scripts** in `gazebo_obstacles/`: random square blocks, or a random maze of square blocks on the 9 pillars of `turtlebot3_world`.
- **`simulation.launch.py`**: starts Gazebo, Nav2, RViz, the initial pose, the obstacles, and the replanner with one command.

Package details, parameters, and how to run each part on its own are in [src/README.md](src/README.md).

## Repository layout

| Path | Contents |
|---|---|
| `src/adaptive_path_replanner/` | The replanner node, launch file, and Nav2 parameters (`config/burger.yaml`) |
| `gazebo_obstacles/` | `wall.sdf`, `spawn_random_walls.py`, `spawn_pillar_maze.py` |
| `turtlebot3.repos` | The upstream ROBOTIS repositories, pinned to tested commits |
| `SETUP.md`, `scripts/setup.sh` | Setup guide and setup script |

The upstream ROBOTIS packages are not stored in this repository. The setup clones them from `turtlebot3.repos`.

## Setup (Ubuntu 22.04)

Full instructions, including how to install ROS 2 Humble, Gazebo and Nav2, are in **[SETUP.md](SETUP.md)**. The short version:

```bash
git clone https://github.com/anishtoppo55/turtlebot_nav2.git
cd turtlebot_nav2
./scripts/setup.sh                 # installs packages (sudo), fetches upstream code, builds

source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch adaptive_path_replanner simulation.launch.py
```
When Gazebo and RViz open, click **Nav2 Goal** in RViz and pick a point on the map.

## Launch options

Pass options to `simulation.launch.py` as `name:=value`, for example:
```bash
ros2 launch adaptive_path_replanner simulation.launch.py layout:=maze maze_walls:=6 maze_seed:=3
ros2 launch adaptive_path_replanner simulation.launch.py num_walls:=5 wall_length:=0.4
ros2 launch adaptive_path_replanner simulation.launch.py layout:=none stuck_timeout:=15.0
```

### Robot and simulation

| Option | Default | What it does |
|---|---|---|
| `use_sim_time` | `true` | Use the Gazebo clock. Keep `true` in simulation. |
| `x_pose`, `y_pose` | `-2.0`, `-0.5` | Where the robot spawns in Gazebo (m). Obstacles keep clear of this point. |
| `yaw_pose` | `0.0` | Robot heading for the initial pose (rad). |
| `set_initial_pose` | `true` | Send the robot's pose to AMCL automatically. Set `false` to click **2D Pose Estimate** in RViz instead. |
| `map` | bundled `turtlebot3_world` map | Map file (`.yaml`) for Nav2. |
| `params_file` | `config/burger.yaml` | Nav2 parameter file. |

### Obstacles

| Option | Default | What it does |
|---|---|---|
| `layout` | `random` | `random` blocks, `maze` on the 9 pillars, or `none`. |
| `walls_delay` | `8.0` | Seconds after start before the obstacles spawn. |
| `obstacles_dir` | `gazebo_obstacles/` in this repo | Folder with `wall.sdf` and the spawn scripts. |

With `layout:=random`:

| Option | Default | What it does |
|---|---|---|
| `num_walls` | `3` | Number of square blocks (`0` = none). |
| `wall_length` | size in `wall.sdf` (0.25 m) | Side of each block (m). |
| `wall_seed` | random | Seed for a repeatable layout. |

With `layout:=maze`:

| Option | Default | What it does |
|---|---|---|
| `maze_walls` | `4` | Exact number of blocks. Up to 8 go between pillars, the rest outside the outer pillars. `-1` uses `maze_openness` and `maze_spokes` instead. |
| `maze_block_size` | size in `wall.sdf` (0.25 m) | Side of each block (m), at most 0.78 (the gap between pillars). |
| `maze_seed` | random | Seed for a repeatable maze. |
| `maze_openness` | `0.0` | With `maze_walls:=-1`: chance (0–1) of opening each extra gap. |
| `maze_spokes` | `3` | With `maze_walls:=-1`: number of blocks outside the outer pillars (0–12). |

### Adaptive path replanner

| Option | Default | What it does |
|---|---|---|
| `initial_inflation` | `0.50` | Global costmap inflation radius at start (m). Must equal `inflation_radius` under `global_costmap` in `params_file`. |
| `inflation_step` | `0.10` | How much each recovery raises the inflation (m). |
| `max_inflation` | `0.80` | Highest inflation the node will set (m). |
| `stuck_timeout` | `8.0` | Seconds without progress before a recovery starts. |
| `clear_global_costmap` | `false` | Clear the global costmap after each raise. This also erases obstacles the lidar has seen. |
| `max_similar_path_retries` | `1` | Extra raises before the node decides there is no other route. |

The node's other parameters, such as `recovery_cooldown` and `comparison_delay`, aren't launch options. Set them by running the node on its own; see [src/README.md](src/README.md#5-adaptive-path-replanner).

`--show-args` also lists options from the included Gazebo and Nav2 launch files (for example `verbose`, `pause`, `autostart`, `log_level`). Those are passed through but rarely needed.

## Run each part individually
These steps start the same things as `simulation.launch.py`, one terminal at a time. That lets you restart a single part without restarting everything. Run this setup first in every terminal:
```bash
cd <workspace>
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```
Start the parts in order. Each one depends on the parts above it.

### 1. Gazebo and the robot
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
This starts the Gazebo server and window, `robot_state_publisher`, and spawns the burger at (-2.0, -0.5). Add `x_pose:=<x> y_pose:=<y>` to spawn it somewhere else.

To start the pieces of this launch file separately instead:
```bash
ros2 launch gazebo_ros gzserver.launch.py \
  world:=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
ros2 launch gazebo_ros gzclient.launch.py                                 # Gazebo window (optional)
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=-2.0 y_pose:=-0.5
```

### 2. Nav2 and RViz
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```
This loads the bundled `turtlebot3_world` map and `param/humble/burger.yaml`, and opens RViz. Use `map:=<file.yaml>` or `params_file:=<file.yaml>` to change them. Wait for `Managed nodes are active` twice in the output (once for localization, once for navigation).

### 3. Initial pose
Either click **2D Pose Estimate** in RViz, or publish the robot's spawn pose:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}, orientation: {w: 1.0}}, \
  covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"
```
AMCL ignores this until it is active. Check with `ros2 lifecycle get /amcl` (it should print `active`).

### 4. Obstacles (optional)
Random square blocks:
```bash
python3 gazebo_obstacles/spawn_random_walls.py --count 4 --size 0.4
python3 gazebo_obstacles/spawn_random_walls.py --clear
```
Or the maze on the 9 pillars:
```bash
python3 gazebo_obstacles/spawn_pillar_maze.py --seed 3
python3 gazebo_obstacles/spawn_pillar_maze.py --clear
```
Both scripts accept `--help`. If the robot isn't at its spawn pose any more, pass `--robot-x` and `--robot-y`, so no wall lands on it.

### 5. Adaptive path replanner
Nav2 must be running, and the active BT must replan periodically. The default Nav2 NavigateToPose BT does this with a RateController around ComputePathToPose.

```bash
ros2 run adaptive_path_replanner adaptive_path_replanner --ros-args \
  -p use_sim_time:=true \
  -p initial_inflation:=0.50 \
  -p inflation_step:=0.10 \
  -p max_inflation:=1.0 \
  -p min_progress_distance:=0.5 \
  -p min_path_deviation:=0.7 \
  -p max_adaptations_per_goal:=6 \
  -p max_similar_path_retries:=1 \
  -p clear_global_costmap:=false \
  -p stuck_timeout:=15.0 \
  -p recovery_cooldown:=10.0 \
  -p comparison_delay:=4.0
```
`initial_inflation` must equal the global costmap's `inflation_radius` in `config/burger.yaml` (0.50 m); with a different value the first "raise" can lower the inflation instead. Use `use_sim_time:=true` in Gazebo and `false` on the real robot. The robot counts as stuck when it hasn't moved `min_progress_distance` metres within `stuck_timeout` seconds. When the goal ends, the node restores the inflation radius to `initial_inflation`.

Each stuck detection starts one recovery, and `max_adaptations_per_goal` limits how many recoveries a goal gets. Within a recovery, the node raises inflation by `inflation_step`. If the new path differs by at least `min_path_deviation`, the recovery succeeds and the higher inflation stays. Otherwise the node raises it up to `max_similar_path_retries` more times. If the path still doesn't change, the node logs `No alternative route found` as a warning and undoes that recovery's raises. This is expected in places with only one way through, such as maze cells.

### 6. Send a goal
Click **Nav2 Goal** in RViz, or send one from the terminal:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.8, y: 0.5}, orientation: {w: 1.0}}}}"
```

### Checks
Verify actual active parameter names and service names on your system:
```bash
ros2 param list /global_costmap/global_costmap
ros2 service list | grep clear_entirely
ros2 topic info /plan
ros2 topic info /navigate_to_pose/_action/status
```

## Important limitations
- `initial_inflation` must match the global costmap's starting `inflation_layer.inflation_radius`.
- The path comparison is a geometric diagnostic. It does not itself force a path to a particular side of an obstacle.
- Increasing inflation can make narrow passages infeasible. Keep the maximum conservative and verify robot footprint/costmap in RViz.
- If the BT does not replan after a costmap change, use a BT with periodic `ComputePathToPose`; this node does not resend the navigation goal.
- Depending on the installed Nav2 version, the costmap clear service name may differ. Check `ros2 service list`.