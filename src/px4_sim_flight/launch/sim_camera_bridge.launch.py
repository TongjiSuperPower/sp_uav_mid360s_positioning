from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    world   = LaunchConfiguration('world_name').perform(context)
    model   = LaunchConfiguration('model_name').perform(context)
    link    = LaunchConfiguration('link_name').perform(context)
    sensor  = LaunchConfiguration('sensor_name').perform(context)

    gz_image_topic = f'/world/{world}/model/{model}/link/{link}/sensor/{sensor}/image'
    gz_info_topic  = f'/world/{world}/model/{model}/link/{link}/sensor/{sensor}/camera_info'

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_bridge',
        output='screen',
        arguments=[
            f'{gz_image_topic}@sensor_msgs/Image[gz.msgs.Image',
            f'{gz_info_topic}@sensor_msgs/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            (gz_image_topic, '/camera/image_raw'),
            (gz_info_topic,  '/camera/camera_info'),
        ]
    )
    return [bridge_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name',   default_value='default'),
        DeclareLaunchArgument('model_name',   default_value='x500_depth_0'),
        DeclareLaunchArgument('link_name',    default_value='camera_link'),
        DeclareLaunchArgument('sensor_name',  default_value='IMX214'),
        OpaqueFunction(function=launch_setup),
    ])
