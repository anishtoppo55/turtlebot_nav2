# TurtleBot3 Adaptive Path Replanner

A TurtleBot3 (burger) simulation on ROS 2 Humble with Gazebo Classic and Nav2. It adds:

- **`adaptive_path_replanner`**: a node that detects when the robot is stuck and raises the global costmap inflation radius, so Nav2 looks for a different route.
- **Obstacle scripts** in `gazebo_obstacles/`: random walls, or a random maze built on the 9 pillars of `turtlebot3_world`.
- **`simulation.launch.py`**: starts Gazebo, Nav2, RViz, the initial pose, the obstacles, and the replanner with one command.

Package details, parameters, and how to run each part on its own are in [src/README.md](src/README.md).

## Quick setup guide

1. Install [Docker Engine](https://docs.docker.com/engine/install/) with the Compose plugin (Linux desktop with X11 or XWayland).
2. Clone the repository:
   ```bash
   git clone https://github.com/anishtoppo55/turtlebot_nav2.git
   cd turtlebot_nav2
   ```
3. Allow the container to open windows on your desktop (once per login):
   ```bash
   xhost +local:root
   ```
4. Build and start the simulation. The first build downloads ROS 2, Gazebo and Nav2, so it takes a while:
   ```bash
   docker compose up --build
   ```
5. When Gazebo and RViz open, click **Nav2 Goal** in RViz and pick a point on the map.

To use the maze instead of random walls: `LAUNCH_ARGS="layout:=maze" docker compose up`. More options are below.

## Repository layout

| Path | Contents |
|---|---|
| `src/adaptive_path_replanner/` | The replanner node, launch file, and Nav2 parameters (`config/burger.yaml`) |
| `gazebo_obstacles/` | `wall.sdf`, `spawn_random_walls.py`, `spawn_pillar_maze.py` |
| `turtlebot3.repos` | The upstream ROBOTIS repositories, pinned to tested commits |
| `Dockerfile`, `docker-compose*.yml`, `docker/` | Container setup |

The upstream ROBOTIS packages are not stored in this repository. Docker and the native setup both clone them from `turtlebot3.repos`.

## Run with Docker (recommended)

Requirements: Linux with an X11 or XWayland desktop, Docker Engine, and the Docker Compose plugin. The image needs several GB of free disk space.

```bash
git clone https://github.com/anishtoppo55/turtlebot_nav2.git
cd turtlebot_nav2

xhost +local:root            # let the container open windows; once per login
docker compose up --build    # the first build takes a while
```

Gazebo and RViz open on your desktop. Click **Nav2 Goal** in RViz to send the robot somewhere.

Launch options go in `LAUNCH_ARGS`:
```bash
LAUNCH_ARGS="layout:=maze maze_seed:=3" docker compose up
LAUNCH_ARGS="num_walls:=5 wall_length:=0.8" docker compose up
LAUNCH_ARGS="layout:=none" docker compose up
```
List every option with `docker compose run --rm sim ros2 launch adaptive_path_replanner simulation.launch.py --show-args`.

Open another shell in the running container, for example to rebuild the maze or check topics:
```bash
docker compose exec sim bash
python3 gazebo_obstacles/spawn_pillar_maze.py --replace --seed 5
ros2 topic echo /plan --once
```

Stop with `Ctrl+C`, or `docker compose down` from another terminal. To undo the display permission, run `xhost -local:root`.

### Graphics
- **Intel/AMD:** `docker-compose.yml` passes `/dev/dri` to the container. If your machine has no `/dev/dri`, delete the `devices:` lines in `docker-compose.yml`. Rendering then falls back to the CPU and is slower.
- **NVIDIA:** install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), then run
  ```bash
  docker compose -f docker-compose.yml -f docker-compose.nvidia.yml up --build
  ```

### Notes
- The container uses host networking, so ROS 2 tools on the host can see its topics when both use the same `ROS_DOMAIN_ID`. Set it before starting, e.g. `ROS_DOMAIN_ID=30 docker compose up`.
- Don't run Gazebo on the host at the same time. Both would use port 11345.
- The code is copied into the image. After changing files, rebuild with `docker compose up --build`.

## Run natively (Ubuntu 22.04)

Install ROS 2 Humble ([instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)), then:

```bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
git clone https://github.com/anishtoppo55/turtlebot_nav2.git
cd turtlebot_nav2

vcs import src < turtlebot3.repos
cp src/adaptive_path_replanner/config/burger.yaml \
   src/turtlebot3/turtlebot3_navigation2/param/humble/burger.yaml

source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install -y --rosdistro humble --ignore-src --from-paths src
colcon build --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch adaptive_path_replanner simulation.launch.py
```
The `cp` step applies this project's Nav2 inflation settings to the upstream parameter file, which `navigation2.launch.py` uses when you start it on its own. `simulation.launch.py` already uses `config/burger.yaml` directly.
