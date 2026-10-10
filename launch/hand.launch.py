from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="dexh13_left"),
            DeclareLaunchArgument("side", default_value="left"),
            DeclareLaunchArgument("hand_port", default_value=""),
            DeclareLaunchArgument("camera_port", default_value="none"),
            DeclareLaunchArgument("connect_on_start", default_value="true"),
            DeclareLaunchArgument("enable_on_start", default_value="true"),
            DeclareLaunchArgument("init_on_start", default_value="false"),
            DeclareLaunchArgument("control_mode", default_value="position"),
            DeclareLaunchArgument("state_hz", default_value="30.0"),
            DeclareLaunchArgument("tactile_hz", default_value="30.0"),
            DeclareLaunchArgument("press_force_hz", default_value="30.0"),
            DeclareLaunchArgument("force_control_hz", default_value="30.0"),
            Node(
                package="dexh13_hand",
                executable="hand_node",
                name=LaunchConfiguration("name"),
                output="screen",
                parameters=[
                    {
                        "side": LaunchConfiguration("side"),
                        "hand_port": LaunchConfiguration("hand_port"),
                        "camera_port": LaunchConfiguration("camera_port"),
                        "connect_on_start": ParameterValue(
                            LaunchConfiguration("connect_on_start"), value_type=bool
                        ),
                        "enable_on_start": ParameterValue(
                            LaunchConfiguration("enable_on_start"), value_type=bool
                        ),
                        "init_on_start": ParameterValue(
                            LaunchConfiguration("init_on_start"), value_type=bool
                        ),
                        "state_hz": LaunchConfiguration("state_hz"),
                        "tactile_hz": LaunchConfiguration("tactile_hz"),
                        "press_force_hz": LaunchConfiguration("press_force_hz"),
                        "force_control_hz": LaunchConfiguration("force_control_hz"),
                    }
                ],
            ),
        ]
    )
