#!/usr/bin/env bash
# ==============================================================================
# Setup Script for AirSim + PX4 ROS 2 Autonomy Framework
# ==============================================================================
# This script sets up dependencies, fetches official px4-ros2-interface-lib and
# px4_msgs, installs Micro-XRCE-DDS Agent, and builds the ROS 2 workspace.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}"
SRC_DIR="${WORKSPACE_DIR}/src"
EXTERNAL_DIR="${SRC_DIR}/external"

echo "======================================================================"
echo " Starting AirSim + PX4 ROS 2 Autonomy Setup"
echo " Workspace: ${WORKSPACE_DIR}"
echo "======================================================================"

# 1. Check ROS 2 environment
if [ -z "${ROS_DISTRO}" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[1/5] Sourcing ROS 2 Humble..."
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "[1/5] Sourcing ROS 2 Jazzy..."
        source /opt/ros/jazzy/setup.bash
    else
        echo "ERROR: No ROS 2 distribution detected in /opt/ros/ (Humble or Jazzy required)."
        echo "Please install ROS 2 Humble or source your ROS environment first."
        exit 1
    fi
else
    echo "[1/5] Using active ROS 2 distribution: ${ROS_DISTRO}"
fi

# 2. Install required system packages and build tools
echo "[2/5] Checking and installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs

# 3. Check and install Micro-XRCE-DDS-Agent if missing
echo "[3/5] Checking Micro-XRCE-DDS-Agent..."
if ! command -v MicroXRCEAgent &> /dev/null; then
    echo "MicroXRCEAgent not found. Installing via snap or compiling..."
    if command -v snap &> /dev/null; then
        sudo snap install micro-xrce-dds-agent --edge || true
    fi
    if ! command -v MicroXRCEAgent &> /dev/null; then
        echo "Building Micro-XRCE-DDS-Agent from source into /tmp..."
        git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent
        mkdir -p /tmp/Micro-XRCE-DDS-Agent/build && cd /tmp/Micro-XRCE-DDS-Agent/build
        cmake .. && make -j$(nproc)
        sudo make install
        sudo ldconfig /usr/local/lib/
        cd "${WORKSPACE_DIR}"
    fi
else
    echo "MicroXRCEAgent is already installed."
fi

# 4. Fetch official external PX4 ROS 2 libraries into ros2/src/external/
mkdir -p "${EXTERNAL_DIR}"
cd "${EXTERNAL_DIR}"

echo "[4/5] Fetching official px4_msgs and px4-ros2-interface-lib..."

# 4a. px4_msgs (release/1.15)
if [ ! -d "px4_msgs" ]; then
    echo "Cloning px4_msgs (branch: release/1.15)..."
    git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
else
    echo "Updating existing px4_msgs..."
    (cd px4_msgs && git pull || true)
fi

# 4b. px4-ros2-interface-lib (official Auterion / PX4)
if [ ! -d "px4-ros2-interface-lib" ]; then
    echo "Cloning px4-ros2-interface-lib..."
    git clone https://github.com/Auterion/px4-ros2-interface-lib.git
else
    echo "Updating existing px4-ros2-interface-lib..."
    (cd px4-ros2-interface-lib && git pull || true)
fi

# 5. Build Workspace
echo "[5/5] Building ROS 2 workspace with colcon..."
cd "${WORKSPACE_DIR}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "======================================================================"
echo " Setup and Build COMPLETED SUCCESSFULLY!"
echo "======================================================================"
echo "To use this workspace in your shell, run:"
echo "    source ${WORKSPACE_DIR}/install/setup.bash"
echo ""
echo "To run the autonomy stack:"
echo "    ${WORKSPACE_DIR}/run_autonomy.sh [obstacle_avoidance|scanning_patrol|target_guiding|area_search]"
echo "======================================================================"

