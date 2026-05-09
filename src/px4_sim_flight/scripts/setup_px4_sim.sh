#!/bin/bash
set -e

echo "=== PX4 Sim Flight Environment Setup ==="

sudo apt update
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-image

GREP_STR="GZ_SIM_RESOURCE_PATH.*px4_sim_flight/models"
if ! grep -q "$GREP_STR" ~/.bashrc; then
    echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/px4_mid360_ws/src/px4_sim_flight/models' >> ~/.bashrc
    echo "[INFO] Added GZ_SIM_RESOURCE_PATH to ~/.bashrc"
fi

cd ~/px4_mid360_ws
colcon build --packages-select px4_msgs px4_sim_flight --symlink-install

echo "=== Setup Complete ==="
echo "Next steps:"
echo "  1. Place point_lio_ros2 and qr_landing in ~/px4_mid360_ws/src/"
echo "  2. Ensure PX4 v1.16.0 is built in ~/PX4-Autopilot"
echo "  3. Generate marker: python3 ~/px4_mid360_ws/src/px4_sim_flight/scripts/generate_aruco_texture.py"
echo "  4. source ~/.bashrc && source ~/px4_mid360_ws/install/setup.bash"
