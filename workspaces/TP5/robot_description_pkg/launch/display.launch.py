import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory("robot_description_pkg"),
        "urdf",
        "diff_drive_robot.urdf",
    )

    # Get the path to the RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory("robot_description_pkg"),
        "rviz",
        "robot_display.rviz",
    )

    # Read the URDF file content
    with open(urdf_file_path, "r") as file:
        robot_description_content = file.read()

    return LaunchDescription([
        # Node to publish joint states (wheel rotations)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        ),

        # Node to publish the robot’s state (tf2) from the URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}],
        ),

        # RViz2 for visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        ),
    ])