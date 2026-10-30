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

Common launch options:
```bash
ros2 launch adaptive_path_replanner simulation.launch.py layout:=maze maze_seed:=3
ros2 launch adaptive_path_replanner simulation.launch.py num_walls:=5 wall_length:=0.4
ros2 launch adaptive_path_replanner simulation.launch.py layout:=none
ros2 launch adaptive_path_replanner simulation.launch.py --show-args    # list every option
```
