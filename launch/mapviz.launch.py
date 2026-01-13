import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

gps_wpf_dir = get_package_share_directory("navegacion_gps")
mapviz_config_file = os.path.join(gps_wpf_dir, "config", "mapviz_gps.mvc")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    return launch.LaunchDescription([
        declare_use_sim_time_cmd,
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            arguments=["-d", mapviz_config_file],
            parameters=[
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        launch_ros.actions.Node(
            package="navegacion_gps",
            executable="interactive_waypoint_follower",
            name="interactive_waypoint_follower",
            output="screen",
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "gps/fix"),
            ],
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"],
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        )
    ])
