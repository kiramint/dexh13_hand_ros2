"""Publish the DexH13 right-hand URDF and open it in RViz 2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "DH13_right_palm_URDF"


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    urdf_path = os.path.join(share, "urdf", "DH13_right_palm_URDF.urdf")
    with open(urdf_path, encoding="utf-8") as stream:
        robot_description = stream.read()

    return LaunchDescription(
        [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            robot_description, value_type=str
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(share, "rviz", "display.rviz")],
            ),
        ]
    )
