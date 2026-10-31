# Adaptive Path Replanner (ROS 2 Humble)

## What it does
- Watches `/navigate_to_pose/_action/status`, `/odom`, and `/plan`.
- Detects a navigation goal with insufficient robot motion.
- Raises the global costmap inflation radius in bounded increments.
- Optionally clears the global costmap (`clear_global_costmap`, off by default). Clearing also erases walls the lidar has seen that aren't in the static map.
- Compares the next global path with the previous path using symmetric mean nearest-polyline distance.
- If the path stays too similar after `max_similar_path_retries` extra raises, it concludes there is no alternative route. It then restores the inflation from before that recovery and leaves the robot to Nav2's own recovery behaviours.

This is a recovery helper, not a replacement for Nav2's planner/controller. It does not publish velocity commands or guarantee a route exists. Test in simulation first.

## Build
For a first-time setup, follow [SETUP.md](../SETUP.md) or run `./scripts/setup.sh` from the repository root. To rebuild only this package after a change:
```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select adaptive_path_replanner
source install/setup.bash
```
`<workspace>` is the folder you cloned the repository into.

## Run everything with one launch file
Starts Gazebo (turtlebot3_world), Nav2 + RViz, the AMCL initial pose, random walls, and the replanner:
```bash
ros2 launch adaptive_path_replanner simulation.launch.py
ros2 launch adaptive_path_replanner simulation.launch.py num_walls:=4 wall_length:=1.0 wall_seed:=5
ros2 launch adaptive_path_replanner simulation.launch.py num_walls:=0 set_initial_pose:=false
```
List all options with `ros2 launch adaptive_path_replanner simulation.launch.py --show-args`.

### Maze on the 9 pillars
`layout:=maze` builds a random maze from the 12 gaps between neighbouring pillars. Every one of the 4 cells inside the grid stays reachable through exactly one gap.
```bash
ros2 launch adaptive_path_replanner simulation.launch.py layout:=maze
ros2 launch adaptive_path_replanner simulation.launch.py layout:=maze maze_walls:=6 maze_seed:=3
ros2 launch adaptive_path_replanner simulation.launch.py layout:=maze maze_walls:=-1 maze_spokes:=4 maze_openness:=0.2
```
Each gap wall runs from one pillar to the next (0.78 m) and closes that gap; outward walls are 0.6 m long. All maze walls are 0.12 m thick (`maze_wall_thickness`, or `--thickness` in the script; `--spoke-length` sets the outward wall length). The height comes from `wall.sdf`.

`maze_walls` (script: `--walls`) sets the exact number of walls; the default is 4. Up to 8 go between pillars, and any beyond that point outward from the outer pillars. Set it to `-1` to choose walls randomly with `maze_openness` and `maze_spokes` instead.
To change the maze while the simulation is running:
```bash
python3 gazebo_obstacles/spawn_pillar_maze.py --dry-run --seed 3      # preview only
python3 gazebo_obstacles/spawn_pillar_maze.py --replace --seed 3      # swap in a new maze
python3 gazebo_obstacles/spawn_pillar_maze.py --clear                 # remove the maze
```

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
Random walls (default 0.8 m long, 0.12 m thick; change with `--length` and `--thickness`):
```bash
python3 gazebo_obstacles/spawn_random_walls.py --count 4 --length 1.0
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
