from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rovio',
            executable='rovio_node',
            name='rovio',
            output='screen',
            prefix=['valgrind --leak-check=full '] 
        )
    ])
