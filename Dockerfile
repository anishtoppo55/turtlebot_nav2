# TurtleBot3 simulation (ROS 2 Humble, Gazebo Classic, Nav2) with the
# adaptive path replanner.
#
# Build:  docker compose build
# Run:    docker compose up            (see README.md for display setup)

FROM osrf/ros:humble-desktop

ARG WS=/turtlebot_3

# GAZEBO_MODEL_DATABASE_URI is empty so Gazebo doesn't try to download models
# at startup; everything the worlds need is installed locally.
ENV WS=${WS} \
    DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger \
    GAZEBO_MODEL_DATABASE_URI=""

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      git \
      python3-colcon-common-extensions \
      python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

WORKDIR ${WS}

# ---------------------------------------------------------------------------
# 1. Upstream ROBOTIS packages, pinned in turtlebot3.repos
# ---------------------------------------------------------------------------
COPY turtlebot3.repos ./
RUN mkdir -p src \
 && vcs import src < turtlebot3.repos \
 && find src -name .git -type d -prune -exec rm -rf {} +

# This project's Nav2 parameters (inflation radii) replace the upstream file,
# so launching turtlebot3_navigation2 directly uses them too.
COPY src/adaptive_path_replanner/config/burger.yaml \
     src/turtlebot3/turtlebot3_navigation2/param/humble/burger.yaml

# Only the packages the simulation needs. DynamixelSDK, turtlebot3_node,
# cartographer and the manipulator packages are for the real robot or other
# demos, so they are cloned but not built.
ARG UPSTREAM_PACKAGES="turtlebot3_gazebo turtlebot3_navigation2 turtlebot3_description turtlebot3_teleop turtlebot3_msgs"

COPY src/adaptive_path_replanner/package.xml src/adaptive_path_replanner/package.xml

RUN apt-get update \
 && rosdep update --rosdistro humble \
 && rosdep install -y --rosdistro humble --ignore-src --from-paths \
      src/adaptive_path_replanner \
      src/turtlebot3/turtlebot3_navigation2 \
      src/turtlebot3/turtlebot3_description \
      src/turtlebot3/turtlebot3_teleop \
      src/turtlebot3_simulations/turtlebot3_gazebo \
      src/turtlebot3_msgs \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash \
 && colcon build --packages-up-to ${UPSTREAM_PACKAGES} \
 && rm -rf build log

# ---------------------------------------------------------------------------
# 2. This project
# ---------------------------------------------------------------------------
COPY src/adaptive_path_replanner src/adaptive_path_replanner
COPY gazebo_obstacles gazebo_obstacles

RUN source /opt/ros/humble/setup.bash \
 && source install/setup.bash \
 && colcon build --packages-select adaptive_path_replanner \
 && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
 && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "source ${WS}/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "adaptive_path_replanner", "simulation.launch.py"]
