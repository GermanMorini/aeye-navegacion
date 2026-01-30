import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _read_file(path):
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def _build_robot_state_publisher(context):
    use_rsp = LaunchConfiguration("use_robot_state_publisher").perform(context)
    if use_rsp.lower() != "true":
        return []

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)
    use_sim_time_bool = use_sim_time == "True"

    robot_description = _read_file(custom_urdf)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time_bool,
                    "robot_description": robot_description,
                }
            ],
        )
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    sensores_dir = get_package_share_directory("sensores")
    launch_dir = os.path.join(gps_wpf_dir, "launch")
    params_dir = os.path.join(gps_wpf_dir, "config")

    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    collision_monitor_params = os.path.join(params_dir, "collision_monitor.yaml")
    lidar_to_scan_params = os.path.join(params_dir, "pointcloud_to_laserscan.yaml")
    rviz_default = os.path.join(params_dir, "rviz_nav2_full.rviz")
    lidar_default_config = os.path.join(sensores_dir, "config", "rs16.yaml")

    bt_xml = os.path.join(
        params_dir, "navigate_to_pose_w_replanning_and_recovery_no_spin.xml"
    )
    bt_through_poses_xml = os.path.join(
        params_dir, "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": bt_xml,
            "default_nav_through_poses_bt_xml": bt_through_poses_xml,
        },
        convert_types=True,
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_mapviz = LaunchConfiguration("use_mapviz")
    use_navsat = LaunchConfiguration("use_navsat")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    use_frame_id_stripper = LaunchConfiguration("use_frame_id_stripper")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    use_pointcloud_to_laserscan = LaunchConfiguration("use_pointcloud_to_laserscan")
    start_pixhawk = LaunchConfiguration("start_pixhawk")
    start_lidar = LaunchConfiguration("start_lidar")
    launch_web = LaunchConfiguration("launch_web")
    custom_urdf = LaunchConfiguration("custom_urdf")
    lidar_config_path = LaunchConfiguration("lidar_config_path")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true",
    )
    declare_use_robot_state_publisher_cmd = DeclareLaunchArgument(
        "use_robot_state_publisher",
        default_value="True",
        description="Publish TF using robot_state_publisher",
    )
    declare_custom_urdf_cmd = DeclareLaunchArgument(
        "custom_urdf",
        default_value=os.path.join(gps_wpf_dir, "models", "cuatri.urdf"),
        description="Path to custom URDF for TF tree",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RVIZ",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_default,
        description="Path to the RViz config file",
    )
    declare_use_mapviz_cmd = DeclareLaunchArgument(
        "use_mapviz",
        default_value="False",
        description="Whether to start mapviz",
    )
    declare_use_navsat_cmd = DeclareLaunchArgument(
        "use_navsat",
        default_value="True",
        description="Whether to start navsat_transform_node",
    )
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        "use_collision_monitor",
        default_value="True",
        description="Whether to start collision monitor",
    )
    declare_use_frame_id_stripper_cmd = DeclareLaunchArgument(
        "use_frame_id_stripper",
        default_value="False",
        description="Whether to strip model prefixes from sensor and odom frame_ids",
    )
    declare_use_pointcloud_to_laserscan_cmd = DeclareLaunchArgument(
        "use_pointcloud_to_laserscan",
        default_value="True",
        description="Whether to start pointcloud_to_laserscan",
    )
    declare_start_pixhawk_cmd = DeclareLaunchArgument(
        "start_pixhawk",
        default_value="True",
        description="Start sensores Pixhawk driver",
    )
    declare_start_lidar_cmd = DeclareLaunchArgument(
        "start_lidar",
        default_value="True",
        description="Start RS16 LiDAR driver",
    )
    declare_launch_web_cmd = DeclareLaunchArgument(
        "launch_web",
        default_value="False",
        description="Start sensores_web node",
    )
    declare_lidar_config_path_cmd = DeclareLaunchArgument(
        "lidar_config_path",
        default_value=lidar_default_config,
        description="Path to rs16 YAML config",
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "dual_ekf_navsat.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_navsat": use_navsat,
        }.items(),
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
            "--ros-args",
            "-p",
            PythonExpression(["'use_sim_time:=' + str(", use_sim_time, ")"]),
        ],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_rviz),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "mapviz.launch.py")),
        condition=IfCondition(use_mapviz),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    collision_monitor_cmd = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            collision_monitor_params,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(use_collision_monitor),
    )
    collision_monitor_lifecycle_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="collision_monitor_lifecycle_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": True,
                "node_names": ["collision_monitor"],
            }
        ],
        condition=IfCondition(use_collision_monitor),
    )

    frame_id_stripper_cmd = Node(
        package="navegacion_gps",
        executable="frame_id_stripper",
        name="frame_id_stripper",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"imu_in_topic": "/imu/data_raw", "imu_out_topic": "/imu/data"},
            {"gps_in_topic": "/gps/fix_raw", "gps_out_topic": "/gps/fix"},
            {"lidar_in_topic": "/scan_3d_raw", "lidar_out_topic": "/scan_3d"},
            {"odom_in_topic": "/odom_raw", "odom_out_topic": "/odom"},
            {"imu_frame_id": "imu_link"},
            {"gps_frame_id": "gps_link"},
            {"lidar_frame_id": "lidar_link"},
            {"odom_frame_id": "odom"},
            {"base_link_frame_id": "base_footprint"},
        ],
        condition=IfCondition(use_frame_id_stripper),
    )

    lidar_to_scan_cmd = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            lidar_to_scan_params,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"output_qos": "sensor_data"},
        ],
        remappings=[
            ("cloud_in", "/scan_3d"),
            ("scan", "/scan"),
        ],
        condition=IfCondition(use_pointcloud_to_laserscan),
    )

    pixhawk_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensores_dir, "launch", "pixhawk.launch.py")
        ),
        launch_arguments={"launch_web": launch_web}.items(),
        condition=IfCondition(start_pixhawk),
    )

    lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensores_dir, "launch", "rs16.launch.py")
        ),
        launch_arguments={"config_path": lidar_config_path}.items(),
        condition=IfCondition(start_lidar),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_publisher_cmd)
    ld.add_action(declare_custom_urdf_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(declare_use_navsat_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_frame_id_stripper_cmd)
    ld.add_action(declare_use_pointcloud_to_laserscan_cmd)
    ld.add_action(declare_start_pixhawk_cmd)
    ld.add_action(declare_start_lidar_cmd)
    ld.add_action(declare_launch_web_cmd)
    ld.add_action(declare_lidar_config_path_cmd)
    ld.add_action(OpaqueFunction(function=_build_robot_state_publisher))
    ld.add_action(pixhawk_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(collision_monitor_cmd)
    ld.add_action(frame_id_stripper_cmd)
    ld.add_action(lidar_to_scan_cmd)
    ld.add_action(collision_monitor_lifecycle_cmd)

    return ld
