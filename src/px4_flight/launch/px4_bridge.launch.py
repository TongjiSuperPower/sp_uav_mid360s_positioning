from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    
    # 获取配置文件路径
    pkg_share = get_package_share_directory('px4_flight')
    default_config = os.path.join(pkg_share, 'config', 'px4_communication.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to PX4 communication config file'
        ),
        
        # 主飞行节点（包含所有模块）
        Node(
            package='px4_flight',
            executable='main_flight_node',
            name='px4_flight_main',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # 可选：单独启动外部定位桥接（用于调试）
        Node(
            package='px4_flight',
            executable='main_flight_node',
            name='pose_bridge_only',
            output='screen',
            parameters=[config_file, {'mode': 'pose_bridge_only'}],
            remappings=[
                ('/fmu/in/vehicle_visual_odometry', '/fmu/in/vehicle_visual_odometry'),
            ]
        ),
    ])