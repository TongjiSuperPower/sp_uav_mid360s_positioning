#!/bin/bash

gnome-terminal -- bash -c "cd ~/px4_mid360_ws && source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360s_launch.py; exec bash"

sleep 5

gnome-terminal -- bash -c "source ~/px4_mid360_ws/install/setup.bash && ros2 launch fast_lio mapping.launch.py rviz:=false config_file:=mid360s.yaml; exec bash"

sleep 5

gnome-terminal -- bash -c "source ~/px4_mid360_ws/install/setup.bash && ros2 launch mavros px4.launch; exec bash"

sleep 5

gnome-terminal -- bash -c "source ~/px4_mid360_ws/install/setup.bash && ros2 run px4_real_flight odometry_to_vision_pose; exec bash"

sleep 5

gnome-terminal -- bash -c "source ~/px4_mid360_ws/install/setup.bash && ros2 run px4_real_flight flight_monitor; exec bash"

sleep 5

gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash&&source ~/px4_mid360_ws/install/setup.bash&&ros2 launch qr_landing qr_landing.launch.py; exec bash"

sleep 5

gnome-terminal -- bash -c "ros2 run qr_landing qr_variance_monitor_node; exec bash"