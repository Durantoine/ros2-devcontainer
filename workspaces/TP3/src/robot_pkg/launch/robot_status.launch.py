from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_pkg',
            executable='status_publisher',
            name='my_status_publisher'
        ),
        Node(
            package='robot_pkg',
            executable='status_subscriber',
            name='my_status_subscriber'
        ),
    ])