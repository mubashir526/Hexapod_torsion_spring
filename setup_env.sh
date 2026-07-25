#!/usr/bin/env bash
# ==============================================================================
# Setup Environment & Install Dependencies for Legged Robot (sim_robot & cheetah_ros2)
# ==============================================================================
# This script installs all required system packages, Gazebo Harmonic tools,
# Python libraries, ROS 2 control packages, and builds the workspace.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# 1. Check ROS 2 Environment
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "--> Sourcing ROS 2 Humble from /opt/ros/humble/setup.bash..."
        source /opt/ros/humble/setup.bash
    else
        echo "[ERROR] ROS_DISTRO is not set and /opt/ros/humble/setup.bash was not found."
        echo "Please install ROS 2 (e.g. Humble) and source it before running this script."
        exit 1
    fi
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
echo "--> Detected ROS 2 Distro: ${ROS_DISTRO}"

# 2. System Build Tools & Prerequisites
echo "--> Updating system package index and installing build tools..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    curl \
    lsb-release \
    gnupg \
    ffmpeg \
    libeigen3-dev \
    build-essential

# 3. Gazebo Harmonic & ROS 2 Control Packages
echo "--> Installing ROS 2 Control, Xacro, and simulation dependencies..."
sudo apt-get install -y -qq \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs || true

# 4. Install Python Dependencies
echo "--> Installing Python dependencies from requirements.txt..."
if [ -f "requirements.txt" ]; then
    pip3 install --no-cache-dir -r requirements.txt
else
    pip3 install --no-cache-dir \
        "numpy<2.0.0" \
        matplotlib \
        onnxruntime \
        osqp \
        pin \
        scipy \
        opencv-python \
        pandas \
        setuptools
fi

# 5. Build gz_ros2_control Underlay (if missing)
if ! python3 -c "from ament_index_python.packages import get_package_prefix; print(get_package_prefix('gz_ros2_control'))" 2>/dev/null; then
    echo "--> gz_ros2_control package not found in current ROS index. Building underlay in ~/gz_control_ws..."
    GZ_WS="${HOME}/gz_control_ws"
    mkdir -p "${GZ_WS}/src"
    if [ ! -d "${GZ_WS}/src/gz_ros2_control" ]; then
        git clone -b "${ROS_DISTRO}" https://github.com/ros-controls/gz_ros2_control.git "${GZ_WS}/src/gz_ros2_control"
    fi
    cd "${GZ_WS}"
    source /opt/ros/${ROS_DISTRO}/setup.bash
    colcon build --symlink-install
    source "${GZ_WS}/install/setup.bash"
    cd "${SCRIPT_DIR}"
fi

# 6. Initialize & Run Rosdep for Workspace
echo "--> Resolving workspace dependencies with rosdep..."
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init || true
fi
rosdep update
if [ -d "ROS/src" ]; then
    rosdep install --from-paths ROS/src --ignore-src -y --rosdistro "${ROS_DISTRO}" || true
fi

# 7. Build Workspace Packages (sim_robot & cheetah_ros2)
echo "--> Building ROS 2 workspace packages..."
colcon build --packages-select sim_robot cheetah_ros2 --symlink-install

echo ""
echo "=============================================================================="
echo " Setup complete! To start using the workspace, run:"
echo "   source install/setup.bash"
echo "=============================================================================="
