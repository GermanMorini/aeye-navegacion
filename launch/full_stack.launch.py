import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navegacion_dir = get_package_share_directory("navegacion_gps")
    controller_dir = get_package_share_directory("controller_server")
    map_tools_dir = get_package_share_directory("map_tools")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")
    use_navsat = LaunchConfiguration("use_navsat")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    start_pixhawk = LaunchConfiguration("start_pixhawk")
    start_lidar = LaunchConfiguration("start_lidar")
    launch_web = LaunchConfiguration("launch_web")
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    gps_topic = LaunchConfiguration("gps_topic")

    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navegacion_dir, "launch", "real.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "use_mapviz": use_mapviz,
            "use_navsat": use_navsat,
            "use_collision_monitor": use_collision_monitor,
            "start_pixhawk": start_pixhawk,
            "start_lidar": start_lidar,
            "launch_web": launch_web,
        }.items(),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_dir, "launch", "controller_server.launch.py")
        )
    )

    map_tools_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_tools_dir, "launch", "no_go_editor.launch.py")
        ),
        launch_arguments={
            "ws_host": ws_host,
            "ws_port": ws_port,
            "gps_topic": gps_topic,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="False",
                description="Whether to start RViz",
            ),
            DeclareLaunchArgument(
                "use_mapviz",
                default_value="False",
                description="Whether to start mapviz",
            ),
            DeclareLaunchArgument(
                "use_navsat",
                default_value="True",
                description="Whether to start navsat_transform_node",
            ),
            DeclareLaunchArgument(
                "use_collision_monitor",
                default_value="True",
                description="Whether to start collision monitor",
            ),
            DeclareLaunchArgument(
                "start_pixhawk",
                default_value="True",
                description="Start sensores Pixhawk driver",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="True",
                description="Start RS16 LiDAR driver",
            ),
            DeclareLaunchArgument(
                "launch_web",
                default_value="False",
                description="Start sensores_web node",
            ),
            DeclareLaunchArgument(
                "ws_host",
                default_value="0.0.0.0",
                description="WebSocket host for no_go_editor",
            ),
            DeclareLaunchArgument(
                "ws_port",
                default_value="8766",
                description="WebSocket port for no_go_editor",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/gps/fix",
                description="GPS topic for no_go_editor",
            ),
            real_launch,
            controller_launch,
            map_tools_launch,
        ]
    )
