from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tag_origin_publisher',
            executable='tag_origin_publisher_node',
            name='tag_origin_publisher',
            output='screen'
        )
    ])
