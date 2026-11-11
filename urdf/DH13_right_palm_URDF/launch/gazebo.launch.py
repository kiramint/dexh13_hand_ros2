"""Spawn the DexH13 right-hand URDF in Gazebo Sim."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "DH13_right_palm_URDF"
MODEL = "DH13_right_palm_URDF"


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    urdf_path = os.path.join(share, "urdf", f"{MODEL}.urdf")
    with open(urdf_path, encoding="utf-8") as stream:
        robot_description = stream.read()
    gz_sim = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim),
                launch_arguments={"gz_args": "-r empty.sdf"}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": ParameterValue(
                            robot_description, value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name",
                    MODEL,
                    "-topic",
                    "robot_description",
                    "-z",
                    "0.2",
                ],
                output="screen",
            ),
        ]
    )
