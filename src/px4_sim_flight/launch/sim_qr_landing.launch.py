import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('px4_sim_flight')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4',
             PathJoinSubstitution([pkg_share, 'worlds', 'qr_landing.world'])],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': os.path.expanduser('~/px4_mid360_ws/src/px4_sim_flight/models')
        }
    )

    # 官方图像/内参/时钟桥接
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_share, 'config', 'gz_bridge.yaml'])
        }],
        output='screen'
    )

    # 手动位姿桥接（从 PoseVector 提取 x500_depth_0）
    manual_bridge = Node(
        package='px4_sim_flight',
        executable='manual_gz_bridge',
        name='manual_gz_bridge',
        output='screen'
    )

    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c',
             'cd ~/PX4-Autopilot && '
             './build/px4_sitl_default/bin/px4 '
             '-d ./build/px4_sitl_default/etc '
             '-s etc/init.d-posix/rcS '
             '-w ./build/px4_sitl_default/tmp -i 0'],
        output='screen'
    )

    odom_bridge = Node(
        package='px4_sim_flight',
        executable='px4_odom_bridge',
        name='px4_odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_gazebo_truth': True,
            'gazebo_truth_topic': '/x500_depth_0/pose'
        }]
    )

    qr_landing = Node(
        package='qr_landing',
        executable='qr_detector_node',
        name='qr_landing',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/camera/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
            ('/camera/camera_info', '/camera_info')
        ]
    )

    landing_node = Node(
        package='px4_sim_flight',
        executable='offboard_landing_node',
        name='offboard_landing_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'qr_land_params.yaml']),
            {'use_sim_time': True}
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'x500_lidar_cam.rviz'])],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        manual_bridge,
        TimerAction(period=3.0, actions=[px4_sitl]),
        TimerAction(period=6.0, actions=[odom_bridge]),
        TimerAction(period=5.0, actions=[qr_landing]),
        TimerAction(period=8.0, actions=[landing_node]),
        TimerAction(period=2.0, actions=[rviz]),
    ])
