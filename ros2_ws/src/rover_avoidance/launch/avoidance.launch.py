from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    preview_only = LaunchConfiguration("preview_only")
    return LaunchDescription([
        DeclareLaunchArgument("preview_only", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rover_avoidance"), "config", "avoidance.yaml"
            ]),
        ),
        Node(
            package="rover_avoidance",
            executable="rover_avoidance",
            name="rover_avoidance",
            output="screen",
            parameters=[params_file, {"preview_only": ParameterValue(preview_only, value_type=bool)}],
        ),
    ])