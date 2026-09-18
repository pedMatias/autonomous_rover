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
    qos = LaunchConfiguration("qos")
    qos_camera_info = LaunchConfiguration("qos_camera_info")
    approx_sync = LaunchConfiguration("approx_sync")
    approx_sync_max_interval = LaunchConfiguration("approx_sync_max_interval")
    topic_queue_size = LaunchConfiguration("topic_queue_size")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    rgb_transport = LaunchConfiguration("rgb_image_transport")
    depth_transport = LaunchConfiguration("depth_image_transport")
    rgb_relay_topic = [rgb_topic, "_relay"]
    depth_relay_topic = [depth_topic, "_relay"]

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rtabmap_launch"),
                "launch",
                "rtabmap.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "localization": LaunchConfiguration("localization"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": LaunchConfiguration("rviz"),
            "frame_id": base_frame,
            "map_frame_id": LaunchConfiguration("map_frame_id"),
            "rgb_topic": rgb_relay_topic,
            "depth_topic": depth_relay_topic,
            "rgb_image_transport": "raw",
            "depth_image_transport": "raw",
            "compressed": "false",
            "qos": qos,
            "qos_camera_info": qos_camera_info,
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "rgbd_sync": "true",
            "approx_rgbd_sync": "true",
            "approx_sync": approx_sync,
            "approx_sync_max_interval": approx_sync_max_interval,
            "topic_queue_size": topic_queue_size,
            "sync_queue_size": sync_queue_size,
            "visual_odometry": LaunchConfiguration("visual_odometry"),
            "icp_odometry": LaunchConfiguration("icp_odometry"),
            "publish_tf_map": "true",
            "database_path": database_path,
            "args": LaunchConfiguration("rtabmap_args"),
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
        DeclareLaunchArgument("camera_roll", default_value="-1.5707963"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="-1.5707963"),
        DeclareLaunchArgument("map_frame_id", default_value="map"),
        DeclareLaunchArgument("rgb_topic", default_value="/oak/rgb/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/oak/stereo/image_raw"),
        DeclareLaunchArgument("rgb_image_transport", default_value="raw"),
        DeclareLaunchArgument("depth_image_transport", default_value="raw"),
        DeclareLaunchArgument("qos", default_value="0"),
        DeclareLaunchArgument("qos_camera_info", default_value="0"),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/oak/rgb/camera_info"
        ),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("approx_sync", default_value="true"),
        DeclareLaunchArgument("approx_sync_max_interval", default_value="0.20"),
        DeclareLaunchArgument("topic_queue_size", default_value="30"),
        DeclareLaunchArgument("sync_queue_size", default_value="30"),
        DeclareLaunchArgument("visual_odometry", default_value="false"),
        DeclareLaunchArgument("icp_odometry", default_value="false"),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value=(
                "--Reg/Force3DoF true "
                "--RGBD/NeighborLinkRefining true "
                "--Reg/Strategy 0 "
                "--Optimizer/Strategy 2 "
                "--RGBD/ProximityBySpace true "
                "--RGBD/ProximityByTime false"
            ),
        ),
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("rtabmap_viz", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="image_transport",
            executable="republish",
            name="republish_rgb_jazzy",
            output="screen",
            parameters=[
                {"in_transport": rgb_transport, "out_transport": "raw"}
            ],
            remappings=[
                (["in/", rgb_transport], [rgb_topic, "/", rgb_transport]),
                ("out", rgb_relay_topic),
            ],
        ),
        Node(
            package="image_transport",
            executable="republish",
            name="republish_depth_jazzy",
            output="screen",
            parameters=[
                {"in_transport": depth_transport, "out_transport": "raw"}
            ],
            remappings=[
                (["in/", depth_transport], [depth_topic, "/", depth_transport]),
                ("out", depth_relay_topic),
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
        rtabmap_launch,
    ])