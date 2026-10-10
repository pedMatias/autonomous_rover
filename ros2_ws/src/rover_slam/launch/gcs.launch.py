from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pi_host = LaunchConfiguration("pi_host")
    depth_topic = LaunchConfiguration("depth_topic")
    depth_preview_topic = LaunchConfiguration("depth_preview_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    package_share = FindPackageShare("rover_slam")

    live_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "live_bringup.launch.py"])
        ),
        launch_arguments={
            "pi_host": pi_host,
            "enable_lidar": LaunchConfiguration("enable_lidar"),
            **{
                f"laser_{axis}": LaunchConfiguration(f"laser_{axis}")
                for axis in ("x", "y", "z", "roll", "pitch", "yaw")
            },
        }.items(),
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
    )

    video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        parameters=[{"port": 8080}],
    )

    depth_preview = Node(
        package="rover_slam",
        executable="gcs_depth_preview",
        name="gcs_depth_preview",
        output="screen",
        parameters=[
            {
                "depth_topic": depth_topic,
                "preview_topic": depth_preview_topic,
            }
        ],
    )

    pointcloud_preview = Node(
        package="rover_slam",
        executable="gcs_pointcloud_preview",
        name="gcs_pointcloud_preview",
        output="screen",
        parameters=[
            {
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "pointcloud_topic": pointcloud_topic,
            }
        ],
    )

    dashboard_server = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "http.server",
            "8000",
            "--directory",
            PathJoinSubstitution([package_share, "gcs"]),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("pi_host", default_value="192.168.0.12"),
        DeclareLaunchArgument("enable_lidar", default_value="false"),
        *[
            DeclareLaunchArgument(f"laser_{axis}", default_value="")
            for axis in ("x", "y", "z", "roll", "pitch", "yaw")
        ],
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/depth/image_raw/compressedDepth",
        ),
        DeclareLaunchArgument(
            "depth_preview_topic",
            default_value="/gcs/depth_preview/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/oak/rgb/camera_info",
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/gcs/pointcloud",
        ),
        live_bringup,
        depth_preview,
        pointcloud_preview,
        rosbridge,
        video_server,
        dashboard_server,
    ])