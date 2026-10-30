#!/usr/bin/env python3
"""
Start the full TurtleBot3 simulation stack with the adaptive path replanner.

Starts:
  1. Gazebo with turtlebot3_world and the robot     (turtlebot3_gazebo)
  2. Nav2 + RViz with the turtlebot3_world map      (turtlebot3_navigation2)
  3. The initial pose for AMCL, once AMCL is active (optional)
  4. Obstacles from gazebo_obstacles/wall.sdf       (optional)
       layout:=random  random walls (spawn_random_walls.py)
       layout:=maze    maze on the 9 pillars (spawn_pillar_maze.py)
       layout:=none    no extra walls
  5. The adaptive path replanner

Usage:
  ros2 launch adaptive_path_replanner simulation.launch.py
  ros2 launch adaptive_path_replanner simulation.launch.py num_walls:=4 wall_length:=0.8
  ros2 launch adaptive_path_replanner simulation.launch.py layout:=maze maze_seed:=3
  ros2 launch adaptive_path_replanner simulation.launch.py num_walls:=0 set_initial_pose:=false
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The TurtleBot3 launch files read TURTLEBOT3_MODEL when they are loaded.
os.environ.setdefault("TURTLEBOT3_MODEL", "burger")

# This file is installed to <ws>/install/adaptive_path_replanner/share/
# adaptive_path_replanner/launch, so the workspace root is five levels up.
PKG_SHARE = get_package_share_directory("adaptive_path_replanner")
WORKSPACE_ROOT = os.path.abspath(os.path.join(PKG_SHARE, "..", "..", "..", ".."))
DEFAULT_OBSTACLES_DIR = os.path.join(WORKSPACE_ROOT, "gazebo_obstacles")


def initial_pose_action(context):
    """Wait until AMCL is active, then publish /initialpose once."""
    if LaunchConfiguration("set_initial_pose").perform(context).lower() != "true":
        return []

    x = float(LaunchConfiguration("x_pose").perform(context))
    y = float(LaunchConfiguration("y_pose").perform(context))
    yaw = float(LaunchConfiguration("yaw_pose").perform(context))

    covariance = [0.0] * 36
    covariance[0] = 0.25    # x
    covariance[7] = 0.25    # y
    covariance[35] = 0.068  # yaw

    msg = (
        "{header: {frame_id: map}, pose: {pose: {"
        f"position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}"
        f"}}, covariance: {covariance}}}}}"
    )

    script = (
        "echo '[initial_pose] waiting for AMCL to become active...'; "
        "until ros2 lifecycle get /amcl 2>/dev/null | grep -q '^active'; "
        "do sleep 1; done; "
        "ros2 topic pub --once -w 1 /initialpose "
        "geometry_msgs/msg/PoseWithCovarianceStamped "
        f"\"{msg}\" && echo '[initial_pose] initial pose set.'"
    )

    return [
        ExecuteProcess(
            cmd=["bash", "-c", script],
            name="set_initial_pose",
            output="screen",
        )
    ]


def obstacles_script(context, name):
    obstacles_dir = LaunchConfiguration("obstacles_dir").perform(context)
    script = os.path.join(obstacles_dir, name)
    if not os.path.isfile(script):
        raise RuntimeError(
            f"Obstacle script not found: {script}. "
            "Set obstacles_dir:=<path to gazebo_obstacles>."
        )
    return obstacles_dir, script


def obstacles_action(context):
    """Spawn the obstacles chosen by the `layout` argument."""
    layout = LaunchConfiguration("layout").perform(context).lower()

    if layout == "none":
        return []
    if layout == "maze":
        return pillar_maze_action(context)
    if layout == "random":
        return random_walls_action(context)

    raise RuntimeError(f"Unknown layout '{layout}'. Use random, maze or none.")


def pillar_maze_action(context):
    """Build a maze on the 9 pillars with gazebo_obstacles/spawn_pillar_maze.py."""
    obstacles_dir, script = obstacles_script(context, "spawn_pillar_maze.py")

    cmd = [
        "python3", script,
        "--sdf", os.path.join(obstacles_dir, "wall.sdf"),
        "--openness", LaunchConfiguration("maze_openness").perform(context),
        "--spokes", LaunchConfiguration("maze_spokes").perform(context),
        "--robot-x", LaunchConfiguration("x_pose").perform(context),
        "--robot-y", LaunchConfiguration("y_pose").perform(context),
        "--timeout", "60",
    ]

    seed = LaunchConfiguration("maze_seed").perform(context)
    if seed:
        cmd += ["--seed", seed]

    return [ExecuteProcess(cmd=cmd, name="spawn_pillar_maze", output="screen")]


def random_walls_action(context):
    """Spawn random walls with gazebo_obstacles/spawn_random_walls.py."""
    count = int(LaunchConfiguration("num_walls").perform(context))
    if count <= 0:
        return []

    obstacles_dir, script = obstacles_script(context, "spawn_random_walls.py")

    cmd = [
        "python3", script,
        "--count", str(count),
        "--sdf", os.path.join(obstacles_dir, "wall.sdf"),
        "--robot-x", LaunchConfiguration("x_pose").perform(context),
        "--robot-y", LaunchConfiguration("y_pose").perform(context),
        "--timeout", "60",
    ]

    length = LaunchConfiguration("wall_length").perform(context)
    if length:
        cmd += ["--length", length]

    seed = LaunchConfiguration("wall_seed").perform(context)
    if seed:
        cmd += ["--seed", seed]

    return [ExecuteProcess(cmd=cmd, name="spawn_random_walls", output="screen")]


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    gazebo_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_world.launch.py",
    )

    nav2_launch = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "launch",
        "navigation2.launch.py",
    )

    default_map = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "map",
        "map.yaml",
    )

    # config/burger.yaml is turtlebot3_navigation2's humble burger.yaml with
    # this project's inflation radii (local 0.35 m, global 0.50 m). Other
    # robot models use the upstream file.
    model_params = os.environ["TURTLEBOT3_MODEL"] + ".yaml"
    bundled_params = os.path.join(PKG_SHARE, "config", model_params)
    default_params = (
        bundled_params
        if os.path.isfile(bundled_params)
        else os.path.join(
            get_package_share_directory("turtlebot3_navigation2"),
            "param",
            "humble",
            model_params,
        )
    )

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="-2.0",
                              description="Robot spawn x in Gazebo"),
        DeclareLaunchArgument("y_pose", default_value="-0.5",
                              description="Robot spawn y in Gazebo"),
        DeclareLaunchArgument("yaw_pose", default_value="0.0",
                              description="Robot yaw for the AMCL initial pose (rad)"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("set_initial_pose", default_value="true",
                              description="Publish the AMCL initial pose automatically"),
        DeclareLaunchArgument("layout", default_value="random",
                              description="Obstacles to add: random, maze or none"),
        DeclareLaunchArgument("maze_seed", default_value="",
                              description="Random seed for a repeatable maze"),
        DeclareLaunchArgument("maze_openness", default_value="0.0",
                              description="Chance (0-1) to open extra maze gaps"),
        DeclareLaunchArgument("maze_spokes", default_value="3",
                              description="Outward walls from the outer pillars (0-12)"),
        DeclareLaunchArgument("num_walls", default_value="3",
                              description="Random walls to spawn (0 = none)"),
        DeclareLaunchArgument("wall_length", default_value="",
                              description="Wall length in m (empty = size in wall.sdf)"),
        DeclareLaunchArgument("wall_seed", default_value="",
                              description="Random seed for a repeatable wall layout"),
        DeclareLaunchArgument("walls_delay", default_value="8.0",
                              description="Seconds to wait before spawning walls"),
        DeclareLaunchArgument("obstacles_dir", default_value=DEFAULT_OBSTACLES_DIR,
                              description="Folder with wall.sdf and spawn_random_walls.py"),
        DeclareLaunchArgument("initial_inflation", default_value="0.50",
                              description="Must match global inflation_radius in params_file"),
        DeclareLaunchArgument("inflation_step", default_value="0.10"),
        DeclareLaunchArgument("max_inflation", default_value="0.80"),
        DeclareLaunchArgument("stuck_timeout", default_value="8.0"),
        DeclareLaunchArgument("clear_global_costmap", default_value="false",
                              description="Clear the global costmap after raising "
                                          "inflation (erases lidar-seen walls)"),
        DeclareLaunchArgument("max_similar_path_retries", default_value="1",
                              description="Extra raises before concluding there is "
                                          "no alternative route"),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "x_pose": x_pose,
            "y_pose": y_pose,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    replanner = Node(
        package="adaptive_path_replanner",
        executable="adaptive_path_replanner",
        name="adaptive_path_replanner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "initial_inflation": LaunchConfiguration("initial_inflation"),
            "inflation_step": LaunchConfiguration("inflation_step"),
            "max_inflation": LaunchConfiguration("max_inflation"),
            "stuck_timeout": LaunchConfiguration("stuck_timeout"),
            "clear_global_costmap": LaunchConfiguration("clear_global_costmap"),
            "max_similar_path_retries": LaunchConfiguration("max_similar_path_retries"),
        }],
    )

    walls = TimerAction(
        period=LaunchConfiguration("walls_delay"),
        actions=[OpaqueFunction(function=obstacles_action)],
    )

    return LaunchDescription(
        arguments
        + [
            SetEnvironmentVariable(
                "TURTLEBOT3_MODEL", os.environ["TURTLEBOT3_MODEL"]
            ),
            gazebo,
            navigation,
            OpaqueFunction(function=initial_pose_action),
            walls,
            replanner,
        ]
    )
