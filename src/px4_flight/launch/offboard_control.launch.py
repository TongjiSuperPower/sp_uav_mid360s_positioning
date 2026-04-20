from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 首先启动雷达和定位
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('px4_flight'), 'launch'),
                '/bringup_mid360.launch.py'
            ]),
            launch_arguments={'record_bag': 'false'}.items()
        ),
        
        # 启动Point-LIO（假设已安装）
        Node(
            package='point_lio_ros2',
            executable='point_lio_node',
            name='point_lio',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('point_lio_ros2'),
                'config', 'mid360.yaml'
            )],
            remappings=[
                ('/livox/lidar', '/livox/lidar'),
                ('/livox/imu', '/livox/imu'),
            ]
        ),
        
        # 启动PX4桥接
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('px4_flight'), 'launch'),
                '/px4_bridge.launch.py'
            ])
        ),
        
        # 启动Offboard控制器
        Node(
            package='px4_flight',
            executable='main_flight_node',
            name='offboard_controller',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('px4_flight'),
                'config', 'flight_modes.yaml'
            )]
        ),
    ])