import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    pkg_qr = get_package_share_directory('qr_landing')
    qr_params = os.path.join(pkg_qr, 'config', 'qr_params.yaml')

    return LaunchDescription([
        # 1. D435 RealSense 驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            ]),
            launch_arguments={
                'align_depth.enable': 'true',
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
            }.items()
        ),

        # 2. 静态 TF：base_link → camera_link（下视安装，已验证正确）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435_base_tf',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '-0.12',
                '--qx', '-0.7071067811865476',
                '--qy', '0.0',
                '--qz', '0.7071067811865476',
                '--qw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        ),

        # 3. 二维码检测节点（3×3 矩阵融合 + 四阶 EMA）
        Node(
            package='qr_landing',
            executable='qr_detector_node',
            name='qr_detector',
            parameters=[qr_params],
            output='screen'
        ),

        # 4. 坐标转换节点
        Node(
            package='qr_landing',
            executable='coordinate_transformer_node',
            name='qr_transformer',
            output='screen'
        ),

        # 5. 测试可视化节点（发布 /qr_landing/qr_position + /qr_landing/qr_euler）
        Node(
            package='qr_landing',
            executable='qr_test_visualizer_node',
            name='qr_test_visualizer',
            output='screen'
        ),

        # 6. 图像查看器（可选，调试用；若机载电脑无显示器可注释掉）
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            arguments=['/qr_landing/test_visualization']
        ),
    ])