import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def validate_mount(context):
    for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
        value = LaunchConfiguration(f"laser_{axis}").perform(context)
        try:
            number = float(value)
        except ValueError as exc:
            raise ValueError(f"laser_{axis} must be a measured numeric mounting value") from exc
        if not math.isfinite(number):
            raise ValueError(f"laser_{axis} must be finite")
    return []


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument("pi_host", default_value="192.168.0.12"),
        DeclareLaunchArgument("lidar_config", default_value=PathJoinSubstitution([
            FindPackageShare("rover_slam"), "config", "lidar.yaml",
        ])),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("laser_frame", default_value="laser_frame"),
    ]
    # These are intentionally required: no guessed mounting transform.
    for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
        declarations.append(DeclareLaunchArgument(f"laser_{axis}"))
    tf_arguments = []
    for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
        tf_arguments.extend([f"--{axis}", LaunchConfiguration(f"laser_{axis}")])
    tf_arguments.extend([
        "--frame-id", LaunchConfiguration("base_frame"),
        "--child-frame-id", LaunchConfiguration("laser_frame"),
    ])
    return LaunchDescription(declarations + [
        OpaqueFunction(function=validate_mount),
        Node(
            package="rover_bridge", executable="rover_lidar_bridge", output="screen",
            parameters=[LaunchConfiguration("lidar_config"), {
                "pi_host": LaunchConfiguration("pi_host"),
                "frame_id": LaunchConfiguration("laser_frame"),
            }],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="lidar_static_transform", output="screen", arguments=tf_arguments,
        ),
    ])
