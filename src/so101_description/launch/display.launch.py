import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")

    urdf_file = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_new_calib.urdf",
    )

    urdf_text = open(urdf_file, "r", encoding="utf-8").read()
    urdf_text = urdf_text.replace(
        'filename="assets/',
        'filename="package://so101_description/assets/',
    )
    robot_description = {"robot_description": urdf_text}

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(use_gui),
        arguments=[urdf_file],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
        arguments=[urdf_file],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
        ]
    )
