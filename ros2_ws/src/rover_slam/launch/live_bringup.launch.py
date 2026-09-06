from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pi_host = LaunchConfiguration("pi_host")
    bridge_config = LaunchConfiguration("bridge_config")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")

    return LaunchDescription([
        DeclareLaunchArgument("pi_host", default_value="192.168.0.12"),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("rover_bridge"),
                "config",
                "rover_bridge.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/oak/rgb/camera_info"
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "camera_frame", default_value="oak_rgb_camera_optical_frame"
        ),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="-1.5707963"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="-1.5707963"),
        Node(
            package="rover_bridge",
            executable="rover_bridge",
            name="rover_bridge",
            output="screen",
            parameters=[
                bridge_config,
                {
                    "pi_host": pi_host,
                    "camera_info_topic": camera_info_topic,
                },
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_static_transform",
            output="screen",
            arguments=[
                "--x",
                LaunchConfiguration("camera_x"),
                "--y",
                LaunchConfiguration("camera_y"),
                "--z",
                LaunchConfiguration("camera_z"),
                "--roll",
                LaunchConfiguration("camera_roll"),
                "--pitch",
                LaunchConfiguration("camera_pitch"),
                "--yaw",
                LaunchConfiguration("camera_yaw"),
                "--frame-id",
                base_frame,
                "--child-frame-id",
                camera_frame,
            ],
        ),
    ])