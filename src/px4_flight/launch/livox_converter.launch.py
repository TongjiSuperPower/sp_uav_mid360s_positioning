from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动原始Livox驱动
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_driver',
            output='screen',
            parameters=[{
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': os.path.join(
                    os.getenv('COLCON_PREFIX_PATH', '/opt/ros/humble'), 
                    '../src/livox_ros_driver2/config/MID360_config.json')
            }]
        ),
        
        # 2. 启动格式转换节点（Mid360S -> VELO16）
        Node(
            package='px4_flight',
            executable='livox_converter',
            name='livox_converter',
            output='screen',
            parameters=[{
                'input_topic': '/livox/lidar',
                'output_topic': '/livox/lidar_converted'
            }]
        ),
        
        # 3. 静态TF（雷达到机体）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_base_tf',
            arguments=['0', '0', '-0.05', '0', '0', '0', 'base_link', 'livox_frame']
        ),
    ])