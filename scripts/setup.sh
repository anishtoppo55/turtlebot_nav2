#!/usr/bin/env bash
# Set up and build this workspace on Ubuntu 22.04 with ROS 2 Humble.
#
# Usage (from the repository root):
#   ./scripts/setup.sh              install everything, then build
#   ./scripts/setup.sh --skip-apt   don't install system packages (no sudo); only fetch and build
#   ./scripts/setup.sh --clean      delete build/, install/ and log/ first
#
# Safe to run again: it skips steps that are already done.

set -euo pipefail

ROS_DISTRO_NAME=humble
SKIP_APT=0
CLEAN=0

for arg in "$@"; do
  case "$arg" in
    --skip-apt) SKIP_APT=1 ;;
    --clean) CLEAN=1 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "Unknown option: $arg (see --help)"; exit 1 ;;
  esac
done

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARNING: %s\033[0m\n' "$*"; }
fail() { printf '\033[1;31mERROR: %s\033[0m\n' "$*"; exit 1; }

# ---------------------------------------------------------------------------
step "Checking the system"
# ---------------------------------------------------------------------------
# shellcheck disable=SC1090,SC1091
. /etc/os-release
if [ "${VERSION_ID:-}" != "22.04" ]; then
  warn "This workspace is tested on Ubuntu 22.04; found ${PRETTY_NAME:-unknown}."
fi
[ -f turtlebot3.repos ] || fail "Run this from the repository (turtlebot3.repos not found in $WS)."

# ---------------------------------------------------------------------------
# System packages
# ---------------------------------------------------------------------------
APT_PACKAGES=(
  "ros-${ROS_DISTRO_NAME}-desktop"
  ros-dev-tools
  "ros-${ROS_DISTRO_NAME}-gazebo-ros-pkgs"
  "ros-${ROS_DISTRO_NAME}-navigation2"
  "ros-${ROS_DISTRO_NAME}-nav2-bringup"
  python3-vcstool
  python3-colcon-common-extensions
  python3-rosdep
  git
)

if [ "$SKIP_APT" -eq 0 ]; then
  step "Installing system packages (needs sudo)"

  if ! apt-cache policy 2>/dev/null | grep -q "packages.ros.org/ros2"; then
    echo "Adding the ROS 2 apt repository..."
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    [ -n "$ROS_APT_SOURCE_VERSION" ] || fail "Could not look up the ros-apt-source release (no internet?)."
    curl -fL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    rm -f /tmp/ros2-apt-source.deb
  fi

  sudo apt-get update
  sudo apt-get install -y "${APT_PACKAGES[@]}"
else
  step "Skipping system packages (--skip-apt)"
  missing=()
  for p in "${APT_PACKAGES[@]}"; do
    dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "install ok installed" || missing+=("$p")
  done
  [ ${#missing[@]} -eq 0 ] || fail "Missing packages: ${missing[*]}. Run without --skip-apt."
  echo "All required system packages are installed."
fi

# setup.bash reads unset variables, so relax -u while sourcing it.
set +u
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

# ---------------------------------------------------------------------------
step "Fetching the upstream ROBOTIS packages (pinned in turtlebot3.repos)"
# ---------------------------------------------------------------------------
mkdir -p src
vcs import --skip-existing src < turtlebot3.repos

# Check that every repository is at the pinned commit.
python3 - <<'EOF'
import subprocess, sys, yaml
repos = yaml.safe_load(open("turtlebot3.repos"))["repositories"]
bad = False
for name, info in repos.items():
    path = f"src/{name}"
    head = subprocess.run(["git", "-C", path, "rev-parse", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    if head != info["version"]:
        bad = True
        print(f"  {name}: at {head[:12] or 'unknown'}, expected {info['version'][:12]}")
    else:
        print(f"  {name}: {head[:12]} OK")
if bad:
    print("WARNING: some repositories are not at the pinned commit. To reset one:\n"
          "  git -C src/<name> fetch && git -C src/<name> checkout <commit>")
EOF

# ---------------------------------------------------------------------------
step "Applying this project's Nav2 parameters"
# ---------------------------------------------------------------------------
UPSTREAM_PARAMS=src/turtlebot3/turtlebot3_navigation2/param/${ROS_DISTRO_NAME}/burger.yaml
PROJECT_PARAMS=src/adaptive_path_replanner/config/burger.yaml
if cmp -s "$PROJECT_PARAMS" "$UPSTREAM_PARAMS"; then
  echo "Already applied."
else
  cp "$PROJECT_PARAMS" "$UPSTREAM_PARAMS"
  echo "Copied $PROJECT_PARAMS -> $UPSTREAM_PARAMS"
  echo "(undo with: git -C src/turtlebot3 checkout -- turtlebot3_navigation2/param/${ROS_DISTRO_NAME}/burger.yaml)"
fi

# ---------------------------------------------------------------------------
step "Installing package dependencies with rosdep"
# ---------------------------------------------------------------------------
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  if [ "$SKIP_APT" -eq 0 ]; then
    sudo rosdep init
  else
    warn "rosdep is not initialised (needs 'sudo rosdep init'); skipping the dependency check."
  fi
fi

if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep update --rosdistro ${ROS_DISTRO_NAME}
  if [ "$SKIP_APT" -eq 0 ]; then
    rosdep install -y --rosdistro ${ROS_DISTRO_NAME} --ignore-src --from-paths src
  else
    rosdep check --rosdistro ${ROS_DISTRO_NAME} --ignore-src --from-paths src \
      || fail "Some dependencies are missing. Run without --skip-apt."
  fi
fi

# A newer setuptools installed with pip in ~/.local overrides the Ubuntu one
# and makes --symlink-install copy Python files instead of linking them.
SETUPTOOLS_PATH=$(python3 -c "import setuptools; print(setuptools.__file__)" 2>/dev/null || true)
case "$SETUPTOOLS_PATH" in
  "$HOME"/.local/*)
    warn "setuptools from pip ($(python3 -c 'import setuptools; print(setuptools.__version__)')) is used instead of Ubuntu's.
         The build works, but after editing Python files you must rebuild.
         To use Ubuntu's version: pip uninstall setuptools" ;;
esac

# ---------------------------------------------------------------------------
step "Building the workspace"
# ---------------------------------------------------------------------------
if [ "$CLEAN" -eq 1 ]; then
  rm -rf build install log
fi
colcon build --symlink-install

# ---------------------------------------------------------------------------
step "Checking the build"
# ---------------------------------------------------------------------------
set +u
# shellcheck disable=SC1090,SC1091
source install/setup.bash
set -u
ros2 pkg executables adaptive_path_replanner | grep -q adaptive_path_replanner \
  || fail "adaptive_path_replanner executable not found after the build."
for pkg in turtlebot3_gazebo turtlebot3_navigation2 adaptive_path_replanner; do
  ros2 pkg prefix "$pkg" >/dev/null || fail "Package $pkg not found after the build."
done
echo "Build OK."

cat <<EOF

Setup complete. To start the simulation:

  cd $WS
  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
  source install/setup.bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch adaptive_path_replanner simulation.launch.py

EOF
