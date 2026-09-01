from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    database_path = LaunchConfiguration("database_path")

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rtabmap_launch"),
                "launch",
                "rtabmap.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "localization": LaunchConfiguration("localization"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": LaunchConfiguration("rviz"),
            "frame_id": base_frame,
            "map_frame_id": LaunchConfiguration("map_frame_id"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "rgbd_sync": "true",
            "approx_rgbd_sync": "true",
            "approx_sync_max_interval": LaunchConfiguration("approx_sync_max_interval"),
            "visual_odometry": "false",
            "icp_odometry": "false",
            "publish_tf_map": "true",
            "database_path": database_path,
            "args": "--delete_db_on_start false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "camera_frame", default_value="oak_rgb_camera_optical_frame"
        ),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("map_frame_id", default_value="map"),
        DeclareLaunchArgument("rgb_topic", default_value="/oak/rgb/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/oak/stereo/image_raw"),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/oak/rgb/camera_info"
        ),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("approx_sync_max_interval", default_value="0.10"),
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("rtabmap_viz", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
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
        rtabmap_launch,
    ])