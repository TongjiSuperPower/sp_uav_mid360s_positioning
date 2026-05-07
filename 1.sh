#!/bin/bash

gnome-terminal -- bash -c "MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 -v6 & MicroXRCEAgent serial --dev /dev/ttyUSB4 -b 921600 -v6; exec bash"

sleep 3

gnome-terminal -- bash -c "cd ~/px4_mid360_ws && source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360s_launch.py; exec bash"

sleep 5

gnome-terminal -- bash -c "source ~/px4_mid360_ws/install/setup.bash && ros2 launch fast_lio mapping.launch.py rviz:=false config_file:=mid360s.yaml; exec bash"

sleep 5

gnome-terminal -- bash -c "source ~/px4_mid360_ws/install/setup.bash && ros2 run px4_flight main_flight_node; exec bash"

sleep 5

gnome-terminal -- bash -c "source ~/px4_mid360_ws/install/setup.bash && ros2 run px4_flight position_monitor --ros-args --params-file ~/px4_mid360_ws/src/px4_flight/config/position_monitor.yaml; exec bash"