#!/bin/bash
# ============================================================================
# TE Node ROS2 Setup Script
# Installs ROS2 Humble and te_arm package on te.local
# ============================================================================

set -e

echo "=============================================="
echo "  TE Node (手) - ROS2 Setup"
echo "=============================================="

# Check if running as root for apt commands
if [ "$EUID" -ne 0 ]; then
    SUDO="sudo"
else
    SUDO=""
fi

echo "[1/6] Installing prerequisites..."
$SUDO apt update
$SUDO apt install -y software-properties-common curl python3-pip

echo "[2/6] Adding ROS2 repository..."
$SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[3/6] Installing ROS2 Humble..."
$SUDO apt update
$SUDO apt install -y ros-humble-desktop \
    ros-humble-robot-state-publisher \
    python3-colcon-common-extensions \
    python3-rosdep

echo "[4/6] Installing Python dependencies..."
pip3 install --user pyserial

echo "[5/6] Initializing rosdep..."
$SUDO rosdep init 2>/dev/null || true
rosdep update

echo "[6/6] Setting up shell..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo ""
echo "=============================================="
echo "  ROS2 Humble installed successfully!"
echo "=============================================="
echo ""
echo "Next steps:"
echo "  1. Log out and back in (or run: source ~/.bashrc)"
echo "  2. Copy te_arm package to this machine"
echo "  3. Build with: cd ~/te_arm/ros2_ws && colcon build"
echo ""

