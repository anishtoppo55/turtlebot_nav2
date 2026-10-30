# Setup guide

How to install everything this workspace needs and build it, so it behaves the same on any machine.

## What you need

| | |
|---|---|
| OS | Ubuntu 22.04 (Jammy), x86-64 or arm64 |
| ROS | ROS 2 Humble, desktop install |
| Simulator | Gazebo Classic 11, installed with `ros-humble-gazebo-ros-pkgs` |
| Navigation | Nav2, installed with `ros-humble-navigation2` and `ros-humble-nav2-bringup` |
| Disk | About 4 GB for ROS, Gazebo and Nav2, plus about 0.5 GB for the workspace |
| Other | Internet access, `sudo`, and a desktop session for the Gazebo and RViz windows |

The upstream ROBOTIS code isn't stored in this repository. The setup clones it from the commits pinned in [`turtlebot3.repos`](turtlebot3.repos), so everyone builds the same versions.

## Quick setup (script)

```bash
git clone https://github.com/anishtoppo55/turtlebot_nav2.git
cd turtlebot_nav2
./scripts/setup.sh
```

The script runs every step in the manual setup below and checks the result. It stops with an error message if a step fails, and you can run it again safely. Options:

| Option | Effect |
|---|---|
| `--skip-apt` | Don't install system packages, which needs no `sudo`. Fails if a required package is missing. |
| `--clean` | Delete `build/`, `install/` and `log/` before building. |

## Manual setup

### 1. Install ROS 2 Humble

These commands follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Skip this step if `/opt/ros/humble` already exists.

```bash
# UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Upgrade first: on Ubuntu 22.04, installing ROS on a system that isn't
# up to date can remove important system packages.
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 2. Install Gazebo, Nav2 and the build tools

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  python3-vcstool python3-colcon-common-extensions python3-rosdep git
```

### 3. Get the code

```bash
git clone https://github.com/anishtoppo55/turtlebot_nav2.git
cd turtlebot_nav2
vcs import src < turtlebot3.repos
```

This clones `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations` and `DynamixelSDK` into `src/`, each at its pinned commit.

### 4. Apply this project's Nav2 parameters

```bash
cp src/adaptive_path_replanner/config/burger.yaml \
   src/turtlebot3/turtlebot3_navigation2/param/humble/burger.yaml
```

`config/burger.yaml` holds this project's settings: the inflation radii and the Regulated Pure Pursuit controller. `simulation.launch.py` reads it directly. The copy makes `navigation2.launch.py` use the same settings when you start Nav2 on its own.

### 5. Install the remaining dependencies

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init        # only once per machine; an "already exists" error is fine
rosdep update
rosdep install -y --rosdistro humble --ignore-src --from-paths src
```

rosdep reads each package's `package.xml` and installs anything still missing. That includes the dependencies of the real-robot and cartographer packages, so the whole workspace builds.

### 6. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

All 17 packages should finish. Warnings in the output from `turtlebot3_gazebo` (CMake) or `turtlebot3_teleop` and `turtlebot3_example` (setuptools) are expected.

### 7. Check the build and run

```bash
ros2 pkg executables adaptive_path_replanner     # should print: adaptive_path_replanner adaptive_path_replanner
export TURTLEBOT3_MODEL=burger
ros2 launch adaptive_path_replanner simulation.launch.py
```

When Gazebo and RViz open, click **Nav2 Goal** in RViz. Launch options and how to run each part on its own are in [src/README.md](src/README.md).

To avoid typing the `source` and `export` lines in every new terminal, add them to `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $PWD/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

## After changing code

Rebuild the package you changed, then source the workspace again:
```bash
colcon build --symlink-install --packages-select adaptive_path_replanner
source install/setup.bash
```
Changes to the scripts in `gazebo_obstacles/` take effect without a rebuild.

## What makes the build reproducible

| Part | How it's fixed |
|---|---|
| This project's code | Stored in this repository |
| ROBOTIS packages (`turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations`, `DynamixelSDK`) | Pinned to exact commits in `turtlebot3.repos`; the script checks each one |
| Nav2 parameters | `src/adaptive_path_replanner/config/burger.yaml`, copied over the upstream file in step 4 |
| ROS 2, Gazebo, Nav2 | Ubuntu packages from the ROS apt repository. These are **not** pinned: apt installs the latest Humble release. Tested with `ros-humble-desktop` 0.10.0 and `ros-humble-navigation2` 1.1.20. |

To record the exact ROS package versions on a working machine, so someone can compare theirs:
```bash
dpkg-query -W 'ros-humble-*' > ros-packages.txt
```

## Troubleshooting

| Problem | Fix |
|---|---|
| `rosdep: your rosdep installation has not been initialized` | Run `sudo rosdep init`, then `rosdep update`. |
| `Package 'adaptive_path_replanner' not found` | Run `source install/setup.bash` in this terminal. |
| `KeyError: 'TURTLEBOT3_MODEL'` when launching `navigation2.launch.py` | `export TURTLEBOT3_MODEL=burger` |
| Code changes have no effect after `colcon build` | A pip-installed setuptools in `~/.local` makes `--symlink-install` copy Python files instead of linking them. Rebuild after each change, or run `pip uninstall setuptools` to use Ubuntu's version. |
| Gazebo hangs at startup on "Getting models from models.gazebosim.org" | Gazebo is trying to download models. Run `export GAZEBO_MODEL_DATABASE_URI=""` before launching. |
| `Address already in use` from Gazebo | Another Gazebo is running. Close it, or check with `pgrep -a gzserver`. |
| Nav2 never becomes active, or AMCL drops scans | Make sure `use_sim_time` is true. `simulation.launch.py` sets it; for `navigation2.launch.py`, pass `use_sim_time:=True`. |
| A repository in `src/` is at a different commit | `git -C src/<name> checkout <commit from turtlebot3.repos>`, or delete `src/<name>` and run `vcs import src < turtlebot3.repos` again. |
