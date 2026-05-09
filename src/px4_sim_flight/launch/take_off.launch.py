from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('px4_sim_flight')
    
    return LaunchDescription([
        Node(
            package='px4_sim_flight',
            executable='offboard_landing_node',
            name='offboard_landing_node',
            output='screen',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'qr_land_params.yaml'])],
            remappings=[
                ('/fmu/in/trajectory_setpoint', '/fmu/in/trajectory_setpoint'),
                ('/fmu/in/vehicle_command', '/fmu/in/vehicle_command'),
                ('/fmu/in/offboard_control_mode', '/fmu/in/offboard_control_mode'),
                ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status'),
                ('/fmu/out/vehicle_local_position', '/fmu/out/vehicle_local_position'),
            ]
        )
    ])
