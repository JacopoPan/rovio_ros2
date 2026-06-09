import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find the share directory of the rovio package
    rovio_share_dir = get_package_share_directory('rovio')

    return LaunchDescription([
        Node(
            package='rovio',
            executable='rovio_node',
            name='rovio',
            output='screen',
            parameters=[
                {'filter_config': os.path.join(rovio_share_dir, 'cfg', 'rovio.info')},
                {'camera0_config': os.path.join(rovio_share_dir, 'cfg', 'euroc_cam0.yaml')},
                {'camera1_config': os.path.join(rovio_share_dir, 'cfg', 'euroc_cam1.yaml')}
            ]
        )
    ])
