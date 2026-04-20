# ~/px4_mid360_ws/src/px4_flight/launch/point_lio_mid360s.launch.py

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = os.path.expanduser('~/px4_mid360_ws')
    
    # Point-LIO源码目录
    point_lio_src = os.path.join(pkg_dir, 'src', 'point_lio')
    
    # 检查路径是否存在
    if not os.path.exists(point_lio_src):
        return LaunchDescription([
            LogInfo(msg=f"ERROR: Point-LIO source not found at {point_lio_src}")
        ])
    
    # 配置文件路径（已提供的mid360s.yaml）
    config_file = os.path.join(point_lio_src, 'config', 'mid360s.yaml')
    
    # 确保配置文件存在
    if not os.path.exists(config_file):
        return LaunchDescription([
            LogInfo(msg=f"ERROR: Config not found at {config_file}")
        ])

    # ==========================================
    # 启动Livox驱动 + 转换节点 + Point-LIO
    # ==========================================

    # 1. Livox Mid360S驱动
    livox_driver_node = Node(
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
            'user_config_path': os.path.join(point_lio_src, '../livox_ros_driver2/config/MID360_config.json')
        }]
    )

    # 2. 格式转换节点（Mid360S -> VELO16）
    livox_converter_node = Node(
        package='px4_flight',
        executable='livox_converter',
        name='livox_converter',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar_converted'
        }]
    )

    # 3. 静态TF（雷达到机体）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_base_tf',
        arguments=['0', '0', '-0.05', '0', '0', '0', 'base_link', 'livox_frame']
    )

    # 4. Point-LIO里程计（关键：使用转换后的话题）
    laser_mapping_node = Node(
        package='point_lio',           # 包名
        executable='pointlio_mapping', # 可执行文件名（根据实际调整）
        name='laserMapping',
        output='screen',
        cwd=point_lio_src,              # 工作目录（让ROOT_DIR正确）
        parameters=[config_file],       # 加载配置文件
        # 话题remapping（确保与配置文件一致）
        remappings=[
            # 输入：转换后的点云
            ('/livox/lidar', '/livox/lidar_converted'),  # 关键：订阅转换后的话题
            ('/livox/imu', '/livox/imu'),                # IMU直接使用
            # 输出：里程计（默认）
            # ('/Odometry', '/point_lio/odom'),          # 如需修改输出话题
        ],
    )

    return LaunchDescription([
        # 设置环境变量（Point-LIO需要）
        SetEnvironmentVariable('ROOT_DIR', point_lio_src + '/'),
        
        LogInfo(msg=f"Point-LIO ROOT_DIR: {point_lio_src}/"),
        LogInfo(msg="Starting Mid360S -> VELO16 -> Point-LIO pipeline"),
        LogInfo(msg="Topics: /livox/lidar -> /livox/lidar_converted -> /point_lio/odom"),
        
        # 启动顺序：驱动 -> 转换 -> TF -> LIO
        livox_driver_node,
        livox_converter_node,
        static_tf_node,
        laser_mapping_node,
    ])