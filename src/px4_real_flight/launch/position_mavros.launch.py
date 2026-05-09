from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Include MAVROS2 px4.launch (XML, already minimized to 4 plugins)
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4.launch'
            ])
        ])
    )

    # Bridge node: FAST-LIO2 /Odometry -> /mavros/vision_pose/pose
    bridge_node = Node(
        package='px4_real_flight',
        executable='odometry_to_vision_pose',
        name='odometry_to_vision_pose',
        output='screen'
    )

    return LaunchDescription([
        mavros_launch,
        bridge_node
    ])
