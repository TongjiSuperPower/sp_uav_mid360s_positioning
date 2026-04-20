from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 阶段1: 启动雷达（延迟0s）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('px4_flight'), 'launch'),
                '/bringup_mid360.launch.py'
            ])
        ),
        
        # 阶段2: 启动Point-LIO（延迟2s，等待雷达就绪）
        TimerAction(
            period=2.0,
            actions=[
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
            ]
        ),
        
        # 阶段3: 启动PX4桥接和控制器（延迟5s，等待LIO初始化）
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('px4_flight'), 'launch'),
                        '/px4_bridge.launch.py'
                    ])
                ),
            ]
        ),
        
        # 阶段4: 启动RViz（延迟6s）
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(
                        get_package_share_directory('px4_flight'),
                        'config', 'visualization.rviz'
                    )]
                ),
            ]
        ),
    ])