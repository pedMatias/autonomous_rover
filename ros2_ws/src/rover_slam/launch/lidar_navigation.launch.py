from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def validate_arming(context):
    values = [
        LaunchConfiguration(name).perform(context).lower()
        for name in ("motion_enabled", "hardware_verified")
    ]
    if any(value not in ("true", "false") for value in values):
        raise ValueError("motion_enabled and hardware_verified must be true or false")
    armed, verified = (value == "true" for value in values)
    if armed and not verified:
        raise ValueError(
            "Arming requires hardware_verified:=true after verifying mounting, "
            "footprint, clocks, scan freshness, wheel limits and stop tests."
        )
    return []


def generate_launch_description():
    config = LaunchConfiguration("params_file")
    common = [config, {"use_sim_time": False}]
    nodes = [
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen", parameters=common + [{"yaml_filename": LaunchConfiguration("map")}],
        ),
        Node(package="nav2_amcl", executable="amcl", name="amcl",
             output="screen", parameters=common),
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen", parameters=common,
            remappings=[("cmd_vel", "/nav2/cmd_vel")],
        ),
        Node(
            package="nav2_planner", executable="planner_server", name="planner_server",
            output="screen", parameters=common,
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server", name="behavior_server",
            output="screen", parameters=common, remappings=[("cmd_vel", "/nav2/cmd_vel")],
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
            output="screen", parameters=common + [{
                "default_nav_to_pose_bt_xml": PathJoinSubstitution([
                    FindPackageShare("rover_slam"), "behavior_trees", "forward_navigation.xml",
                ]),
            }],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lidar_navigation_lifecycle", output="screen",
            parameters=[{"autostart": True, "node_names": [
                "map_server", "amcl", "controller_server", "planner_server",
                "behavior_server", "bt_navigator",
            ]}],
        ),
        Node(
            package="rover_bridge", executable="rover_navigation_gate",
            name="rover_navigation_gate", output="screen",
            parameters=[{
                "motion_enabled": ParameterValue(
                    LaunchConfiguration("motion_enabled"), value_type=bool
                ),
            }],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("pi_host", default_value="192.168.0.12"),
        *[
            DeclareLaunchArgument(f"laser_{axis}")
            for axis in ("x", "y", "z", "roll", "pitch", "yaw")
        ],
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            FindPackageShare("rover_slam"), "config", "nav2_lidar.yaml",
        ])),
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("hardware_verified", default_value="false"),
        OpaqueFunction(function=validate_arming),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("rover_slam"), "launch", "live_bringup.launch.py",
            ])),
            launch_arguments={
                "pi_host": LaunchConfiguration("pi_host"),
                "enable_lidar": "true",
                "cmd_vel_topic": "/cmd_vel_safe",
                **{
                    f"laser_{axis}": LaunchConfiguration(f"laser_{axis}")
                    for axis in ("x", "y", "z", "roll", "pitch", "yaw")
                },
            }.items(),
        ),
        *nodes,
    ])
