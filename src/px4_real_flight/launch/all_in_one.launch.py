from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360s_launch.py'],
            output='screen',
            shell=False,
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py', 
                         'rviz:=false', 'config_file:=mid360s.yaml'],
                    output='screen',
                    shell=False,
                ),
            ]
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='mavros',
                    executable='mavros_node',
                    name='mavros',
                    output='screen',
                    parameters=[{
                        'fcu_url': '/dev/ttyACM0:921600',
                        'gcs_url': '',
                        'conn_timeout': 10.0,
                        'heartbeat_rate': 1.0,
                        'plugin_denylist': [
                            'safety_area', 'image_pub', 'vibration', 'wind_estimation',
                            'rangefinder', 'altitude', 'hil', 'ftp', 'log_transfer',
                            'debug_value', 'vision_speed_estimate', 'landing_target',
                            'mount_control', 'gimbal', 'esc_status', 'rc_io',
                            'actuator_control', 'manual_control', 'follow_target',
                            'geofence', 'home_position', 'mission', 'param',
                            'waypoint', 'rallypoint', 'command', 'global_position',
                            'gps', 'gps_status', 'gps_rtk', 'satellite',
                            'magnetometer', 'pressure', 'temperature',
                            'extended_state', 'sys_status', 'sys_time', 'time',
                            'vfr_hud', 'wind', 'terrain', 'distance_sensor',
                            'companion_process_status', 'onboard_computer_status',
                            'play_tune', 'tune', 'camera', 'video_stream'
                        ],
                        'plugin_allowlist': [
                            'local_position', 'setpoint_position', 'setpoint_velocity',
                            'odometry', 'command', 'state', 'imu'
                        ],
                        'imu/data_raw/send': False,
                        'imu/data/send': True,
                        'imu/data_rate': 50.0,
                        'local_position/pose/send': True,
                        'local_position/pose_rate': 50.0,
                        'local_position/velocity/send': True,
                        'local_position/velocity_rate': 50.0,
                        'local_position/tf/send': False,
                        'odometry/in': 'odometry',
                        'odometry/fcu/odom_parent_frame_des': 'map',
                        'odometry/fcu/odom_child_frame_des': 'base_link',
                        'conn/system_time': False,
                        'conn/timesync': False,
                    }],
                    remappings=[
                        ('/mavros/odometry/in', '/Odometry'),
                    ]
                ),
            ]
        ),
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='px4_real_flight',
                    executable='odometry_to_vision_pose',
                    name='odometry_to_vision_pose',
                    output='screen',
                ),
                Node(
                    package='px4_real_flight',
                    executable='ned_position_listener',
                    name='ned_position_listener',
                    output='screen',
                ),
            ]
        ),
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='px4_real_flight',
                    executable='flight_monitor',
                    name='flight_monitor',
                    output='screen',
                ),
            ]
        ),
    ])
