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

## Running each part on its own

To start or restart one part at a time, use one terminal per step, in this order. Run `source /opt/ros/humble/setup.bash`, `source install/setup.bash` and `export TURTLEBOT3_MODEL=burger` in each terminal first.

| Step | Command |
|---|---|
| 1. Gazebo and the robot | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` |
| 2. Nav2 and RViz | `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True` |
| 3. Initial pose | Click **2D Pose Estimate** in RViz |
| 4. Obstacles (optional) | `python3 gazebo_obstacles/spawn_pillar_maze.py` or `python3 gazebo_obstacles/spawn_random_walls.py --count 4` |
| 5. Replanner | `ros2 run adaptive_path_replanner adaptive_path_replanner --ros-args -p use_sim_time:=true -p initial_inflation:=0.50` |
| 6. Goal | Click **Nav2 Goal** in RViz |

The options for each step, the obstacle script commands (`--replace`, `--clear`, `--dry-run`), and a terminal command for the initial pose are in [src/README.md](src/README.md#run-each-part-individually).
