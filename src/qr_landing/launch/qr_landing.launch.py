import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_qr = get_package_share_directory('qr_landing')
    
    qr_params = os.path.join(pkg_qr, 'config', 'qr_params.yaml')
    d435_extrinsic = os.path.join(pkg_qr, 'config', 'd435_extrinsic.yaml')
    
    import yaml
    with open(d435_extrinsic, 'r') as f:
        ext = yaml.safe_load(f)['static_transform']
    trans = ext['translation']
    rot = ext['rotation']
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='FOLLOW',
            description='Default operation mode: FOLLOW or LAND'
        ),
        
        # D435 RealSense 驱动（去掉 namespace，避免双重 camera 前缀）
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d435_camera',
            # namespace='camera',  # <-- 删除或注释掉此行
            parameters=[{
                'camera_name': 'camera',           # 话题前缀为 /camera/...
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'align_depth.enable': True,        # 关键：启用深度对齐到 RGB
                'publish_tf': True,
                'tf_publish_rate': 10.0,
            }],
            output='screen'
        ),
        
        # Static TF: base_link -> camera_link（下视安装外参）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435_base_tf',
            arguments=[
                str(trans[0]), str(trans[1]), str(trans[2]),
                str(rot[0]), str(rot[1]), str(rot[2]),
                'base_link', 'camera_link'
            ]
        ),
        
        # QR 检测节点
        Node(
            package='qr_landing',
            executable='qr_detector_node',
            name='qr_detector',
            parameters=[qr_params],
            output='screen'
        ),
        
        # 坐标转换节点
        Node(
            package='qr_landing',
            executable='coordinate_transformer_node',
            name='qr_transformer',
            parameters=[{
                'target_frame': 'base_link',
                'global_frame': 'map',
            }],
            output='screen'
        ),
        
        # 降落控制器（双模式）
        Node(
            package='qr_landing',
            executable='landing_controller_node',
            name='landing_controller',
            parameters=[qr_params, {'default_mode': LaunchConfiguration('mode')}],
            output='screen'
        ),
    ])