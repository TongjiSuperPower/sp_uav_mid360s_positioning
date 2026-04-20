from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # 参数声明
    record_bag = LaunchConfiguration('record_bag', default='false')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Record rosbag for testing'
        ),
        
        # Livox Mid360驱动节点
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_driver',
            output='screen',
            parameters=[
                {'xfer_format': 0},  # 点云格式
                {'multi_topic': 0},
                {'data_src': 0},
                {'publish_freq': 10.0},
                {'output_data_type': 0},
                {'frame_id': 'livox_frame'},
                {'lvx_file_path': ''},
                {'user_config_path': os.path.join(
                    os.getenv('COLCON_PREFIX_PATH', '/opt/ros/humble'), 
                    '../src/livox_ros_driver2/config/MID360_config.json')},
            ],
            remappings=[
                ('/livox/lidar', '/livox/lidar'),
                ('/livox/imu', '/livox/imu'),
            ]
        ),
        
        # 静态TF发布（雷达到机体）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_base_tf',
            arguments=[
                '0', '0', '-0.05',  # x, y, z (雷达在IMU下方5cm)
                '0', '0', '0',      # roll, pitch, yaw
                'base_link', 'livox_frame'
            ]
        ),
        
        # 可选：rosbag录制
        Node(
            condition=IfCondition(record_bag),
            package='ros2bag',
            executable='record',
            name='record_bag',
            arguments=['-a', '-o', '/home/user/rosbags/mid360_test']
        ),
    ])